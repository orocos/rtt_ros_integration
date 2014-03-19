#include <cstdlib>
#include <list>
#include <queue>
#include <sstream>
#include <set>

#include <boost/filesystem.hpp>
#include <boost/version.hpp>

#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>
#include <libxml/tree.h>

#include <rtt/RTT.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/Logger.hpp>

#include <ros/package.h>
#include <rospack/rospack.h>

#include <rtt_ros/rtt_ros.h>


bool rtt_ros::import(const std::string& package)
{
  RTT::Logger::In in("ROSService::import(\""+package+"\")");

  boost::shared_ptr<RTT::ComponentLoader> loader = RTT::ComponentLoader::Instance();

  // List of packages which could not be loaded
  std::vector<std::string> missing_packages;

  // Get the dependencies for a given ROS package --> deps
  try {
    // Get the package paths
    std::vector<std::string> ros_package_paths;
    rospack::Rospack rpack;
    rpack.setQuiet(true);
    rpack.getSearchPathFromEnv(ros_package_paths);

    RTT::log(RTT::Debug) << "Loading dependencies for ROS package \""<< package << "\" from: ";
    for(size_t i=0; i<ros_package_paths.size(); i++) { RTT::log(RTT::Debug) << ros_package_paths[i]; }
    RTT::log(RTT::Debug) << RTT::endlog();

    rpack.crawl(ros_package_paths,true);

    if ( ros_package_paths.size() == 0 ) {
      RTT::log(RTT::Error) << "No paths in the ROS_PACKAGE_PATH environment variable! Could not load ROS package \"" << package << "\"" << RTT::endlog();
      return false;
    }

    // Add all rtt_ros/plugin_depend dependencies to package names
    std::vector<std::string> deps_to_import;
    std::vector<std::string> search_paths;
    rpack.setQuiet(true);

    // Read the package.xml for this package
    std::queue<std::string> dep_names;
    dep_names.push(package);
    deps_to_import.push_back(package);

    const xmlChar * rtt_plugin_depend_xpath = xmlCharStrdup("/package/export/rtt_ros/plugin_depend/text()");

    while(!dep_names.empty()) 
    {
      namespace fs = boost::filesystem;

      // Get the next dep name, store the dep path
      std::string dep_name, dep_path;
      dep_name = dep_names.front();
      dep_names.pop();

      bool dep_found = rpack.find(dep_name,dep_path);

      if(!dep_found) {
        RTT::log(RTT::Error) << "Could not find ROS package \""<< dep_name << "\" in ROS_PACKAGE_PATH environment variable." <<RTT::endlog();
        continue;
      }

      // Construct the package.xml path
      fs::path package_xml_path = fs::path(dep_path) / "package.xml";
      bool is_rosbuild_package = false;

      // Check if package.xml file exists
      if(!boost::filesystem::is_regular_file( package_xml_path.string() ) ) {

        // Fall back to manifest.xml for rosbuild packages
        package_xml_path = fs::path(dep_path) / "manifest.xml";
        is_rosbuild_package = true;

        if(!boost::filesystem::is_regular_file( package_xml_path.string() ) ) {
          RTT::log(RTT::Error) << "No package.xml or manifest.xml file for ROS package \""<< dep_name << "\" found at "<<package_xml_path.branch_path() <<RTT::endlog();
          continue;
        }
      }

      // Add package path to the list of search paths
      if (is_rosbuild_package) {
        search_paths.push_back((fs::path(dep_path) / "lib" / "orocos").string());
      }

      // Read in package.xml/manifest.xml
      xmlInitParser();

      // libxml structures
      xmlDocPtr package_doc;
      xmlXPathContextPtr xpath_ctx;
      xmlXPathObjectPtr xpath_obj;

      // Load package.xml
      package_doc = xmlParseFile(package_xml_path.string().c_str());
      xpath_ctx = xmlXPathNewContext(package_doc);

      // Get the text of the rtt_ros <plugin_depend>s
      xpath_obj = xmlXPathEvalExpression(rtt_plugin_depend_xpath, xpath_ctx);

      // Iterate through the nodes
      if(xmlXPathNodeSetIsEmpty(xpath_obj->nodesetval)) {
        RTT::log(RTT::Debug) << "ROS package \""<< dep_name << "\" has no RTT plugin dependencies." <<RTT::endlog();
      } else {
        RTT::log(RTT::Debug) << "ROS package \""<< dep_name << "\" has "<<xpath_obj->nodesetval->nodeNr<<" RTT plugin dependencies." <<RTT::endlog();

        for(int i=0; i < xpath_obj->nodesetval->nodeNr; i++) {
          if(xpath_obj->nodesetval->nodeTab[i]) {
            std::ostringstream oss;
            oss << xmlNodeGetContent(xpath_obj->nodesetval->nodeTab[i]);
            RTT::log(RTT::Debug) << "Found dependency \""<< oss.str() << "\"" <<RTT::endlog();
            dep_names.push(oss.str());

            // Add the dep to the list of deps to import
            deps_to_import.push_back(oss.str());
          }
        }
      }

      xmlXPathFreeObject(xpath_obj);
      xmlXPathFreeContext(xpath_ctx);
      xmlFreeDoc(package_doc);
      xmlCleanupParser();
    }

    // Build path list by prepending paths from search_paths list to the RTT component path in reverse order without duplicates
    std::set<std::string> search_paths_seen;
    std::string path_list = loader->getComponentPath();
    for(std::vector<std::string>::reverse_iterator it = search_paths.rbegin();
        it != search_paths.rend();
        ++it)
    {
      if (search_paths_seen.count(*it)) continue;
      path_list = *it + ":" + path_list;
      search_paths_seen.insert(*it);
    }

    RTT::log(RTT::Debug) << "Attempting to load RTT plugins from "<<deps_to_import.size()<<" packages..." << RTT::endlog();

    // Import each package dependency and the package itself (in deps_to_import[0])
    for(std::vector<std::string>::reverse_iterator it = deps_to_import.rbegin();
        it != deps_to_import.rend();
        ++it)
    {
      // Check if it's already been imported
      if(*it == "rtt_ros" || loader->isImported(*it)) {
        RTT::log(RTT::Debug) << "Package dependency '"<< *it <<"' already imported." << RTT::endlog();
        continue;
      }

      // Import the dependency
      if(loader->import(*it, path_list)) {
        RTT::log(RTT::Debug) << "Importing Orocos components from ROS package \""<<*it<<"\" SUCCEEDED.";
      } else {
        // Temporarily store the name of the missing package
        missing_packages.push_back(*it);
        RTT::log(RTT::Debug) << "Importing Orocos components from ROS package \""<<*it<<"\" FAILED.";
      }
      RTT::log(RTT::Debug) << RTT::endlog();
    }

  } catch(std::string arg) {
    RTT::log(RTT::Debug) << "While processing the dependencies of " << package << ": Dependency is not a ros package: " << arg << RTT::endlog();
    missing_packages.push_back(arg);
  }

  // Report success or failure
  if(missing_packages.size() == 0) { 
    RTT::log(RTT::Info) << "Loaded plugins from ROS package \"" << package << "\" and its dependencies." << RTT::endlog();
  } else {
    RTT::log(RTT::Warning) << "Could not load RTT plugins from the following ROS packages (they might be empty, in which case this message can be ignored): "<< RTT::endlog();
    for(std::vector<std::string>::iterator it = missing_packages.begin();
        it != missing_packages.end();
        ++it)
    {
      RTT::log(RTT::Warning) << " - " << *it<< RTT::endlog();
    }
  }

  return missing_packages.size() == 0;
}
