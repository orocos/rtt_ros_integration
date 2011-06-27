RUBY_VERSION=`ruby --version | awk '{ print $2; }' | sed -e "s/\(.*\..*\)\..*/\1/"`
RUBY_ARCH=`ruby --version | sed -e 's/.*\[\(.*\)\]/\1/'`
export RUBYLIB=`rospack find utilrb`/lib:`rospack find orogen`/lib:`rospack find typelib`/install/lib/ruby/${RUBY_VERSION}/${RUBY_ARCH}:`rospack find typelib`/install/lib/ruby/${RUBY_VERSION}
export GEM_HOME=`rosstack find orocos_toolchain_ros`/.gems
export RUBYOPT=-rubygems
export PATH=`rospack find orogen`/bin:`rosstack find orocos_toolchain_ros`/.gems/bin:$PATH
export TYPELIB_USE_GCCXML=1
export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:`rospack find typelib`/install/lib/pkgconfig:`rospack find utilmm`/install/lib/pkgconfig:`rospack find rtt`/install/lib/pkgconfig:`rospack find rtt_typelib`/install/lib/pkgconfig
