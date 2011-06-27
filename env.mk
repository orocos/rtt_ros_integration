export RUBYLIB:=$(shell rospack find utilrb)/lib:$(shell rospack find orogen)/lib:$(shell rospack find typelib)/bindings/ruby/lib:$(shell rospack find typelib)/lib
export GEM_HOME:=$(shell rosstack find orocos_toolchain_ros)/.gems
export RUBYOPT:=-rubygems
export PATH:=$(shell rospack find orogen)/bin:$(shell rosstack find orocos_toolchain_ros)/.gems/bin:${PATH}
export TYPELIB_USE_GCCXML:=1
export PKG_CONFIG_PATH=$(shell rospack find typelib)/install/lib/pkgconfig:$(shell rospack find utilmm)/install/lib/pkgconfig:$(shell rospack find rtt)/install/lib/pkgconfig
