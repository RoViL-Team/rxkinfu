# makefile for C/C++/CUDA code that uses PCL
# vona 12/2013
# name C files .c, C++ files .cpp, CUDA files .cu
# name header files .h for interface and .hpp for implementation
# supports Ubuntu 12.04 (gcc/g++) and OS X 10.9 (clang and homebrew)

SHELL = /bin/bash

COMMA := ,
EMPTY :=
SPACE := $(EMPTY) $(EMPTY)

default: all

# VT100 colors
vt100red = 31
vt100grn = 32
vt100yel = 33
vt100blu = 34

# generate a colored vt100 message for bash echo -e
# arg1 is vt100 color code
# arg2 is message string
vt100 = "\033[0;$(vt100$(1))m$(2)\033[0m"

# warning and info messages
wmsg = $(info $(shell echo -e $(call vt100,red,WARN) $(1)))
imsg = $(info $(shell echo -e $(call vt100,yel,NOTE) $(1)))

# upcase string
toupper = $(shell echo "$(1)" | tr "[:lower:]" "[:upper:]")

# default project name is project directory name
# define here so it can be used in makefile.project
DEF_PROJECT := $(notdir $(lastword $(shell pwd)))

# default project lib deps are all expected object files not containing a main()
# define here so it can be used in makefile.project, further doc there
# use = not := because WITH_CUDA will be defined in makefile.project
# sort also removes dupes
DEF_PROJECT_LIB_SRCS = \
$(filter-out test% $(BIN_TARGETS:.bin=.cpp) $(BIN_TARGETS:.bin=.c),\
$(wildcard *.cpp *.c $(if $(WITH_CUDA),*.cu)))
DEF_PROJECT_LIB_DEPS = \
$(sort $(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(patsubst %.cu,%.o,\
$(DEF_PROJECT_LIB_SRCS)))))

# standard boost libs to include when WITH_BOOST=1
DEF_BOOST_LIBS := -lboost_serialization -lboost_system \
                  -lboost_filesystem -lboost_thread \
                  -lboost_date_time -lboost_iostreams \
                  -lboost_regex -lboost_mpi

# default PCL version to link against
DEF_PCL_VERSION ?= 1.8

# detect OS and compiler
IS_LINUX := $(shell uname -s | grep -q Linux && echo 1)
IS_DARWIN := $(shell uname -s | grep -q Darwin && echo 1)
IS_X86_64 := $(shell uname -m | grep -q x86_64 && echo 1)
CC_IS_CLANG := $(shell $(CC) --version | grep -q clang && echo 1)
CXX_IS_CLANG := $(shell $(CXX) --version | grep -q clang && echo 1)

# shared library extension
SOEXT := $(if $(IS_DARWIN),dylib,so)

# check for required tools
tool-exists = \
$(shell which $(1) > /dev/null 2>&1 && [ -x `which $(1)` ] && echo 1)

# call pkg-config
# arg1 is cflags, libs, exists, modversion, variable=foo, etc
# arg2 is the package name
# -I is translated to -isystem in cflags
# output is translated to either empty or 1 for exists
# prepends PC_LOCAL to PKG_CONFIG_PATH to supply .pc files locally if needed 
pkg-config = $(strip $(subst -I,$(if $(filter cflags,$(1)),-isystem ,-I),\
  $(shell PKG_CONFIG_PATH=$(PC_LOCAL):$$PKG_CONFIG_PATH pkg-config --$(1) $(2) \
    $(if $(filter exists,$(1)),> /dev/null 2>&1 && echo 1))))

# call pkg-config for a pcl module
# sigh https://github.com/mariusmuja/flann/pull/24
pcl-pc = $(strip $(if $(strip $(2)),\
  $(filter-out -lflann_cpp-gd,\
  $(call pkg-config,$(1),$(foreach m,$(2),pcl_$(m)-$(PCL_VERSION))))))
pcl-cflags = $(call pcl-pc,cflags,$(1))
pcl-libs = $(call pcl-pc,libs,$(1))

# call cmake --find-package which is like pkg-config
# -I is translated to -isystem in COMPILE flags
# output is translated to either empty or 1 for EXIST
# cached because it's slow, run make cacheclean or make realclean to reset
# arg 1 is C or CXX
# arg 2 is COMPILE, LINK, or EXIST
# arg 3 is the module name
cmake-find-pkg = $(call \
  cmake-find-pkg-cached,$(1),$(2),$(3),.cmake-find-package-$(1)-$(2)-$(3).cache)
cmake-find-pkg-cached = \
$(subst -I,$(if $(filter COMPILE,$(2)),-isystem ,-I),$(strip $(shell \
  ( [ -f $(4) ] && cat $(4) ) || \
  ( pushd /tmp > /dev/null && \
    cmake -DCOMPILER_ID=GNU -DLANGUAGE=$(1) -DMODE=$(2) -DNAME=$(3) \
    --find-package | $(if $(filter EXIST,$(2)),\
                          grep -q "$(3) found" && echo 1 | )\
    tee $(4) 2>/dev/null && popd > /dev/null && mv /tmp/$(4) . ) )))

# prevent make from always trying to remake makefiles
makefile.project: ;

# include project specific stuff
-include makefile.project

# look for pkg-config files in project dir by default
PC_LOCAL ?= .

# allows conditional compilation possibly outside of project
HAVE_LIBS += -DHAVE_$(call toupper,$(PROJECT))=1

# allows compilation without pcl
HAVE_LIBS += -DHAVE_PCL=$(if $(WITH_PCL),$(call pcl-pc,exists,common))

# check for object filename collisions
ifneq ($(words $(DEF_PROJECT_LIB_SRCS)),$(words $(DEF_PROJECT_LIB_DEPS)))
NN := $(basename $(DEF_PROJECT_LIB_SRCS))
$(call wmsg,"object filenames will collide: \
$(sort $(foreach N,$(NN),$(if $(filter-out 1,$(words $(filter $(N),$(NN)))),\
$(wildcard $(N).c*))))")
endif

# add a generic library dependency
# arg1 is the package name
# arg2 is an optional varname prefix/suffix, defaults to the upcase of arg1
# the package is added iff WITH_$(2) is defined
# if $(2)_FLAGS and/or $(2)_LIBS are predefined then they are used directly
# else the package is found by one of the following methods:
# WITH_$(2) = pkg-config or 1 : use pkg-config foo
# WITH_$(2) = cmake           : use cmake --find-package foo
# WITH_$(2) = path            : use -I $(WITH_FOO) -L $(WITH_FOO)/foo -lfoo
# NOTE: indent here with spaces only to avoid confusing make
gendep = $(eval $(call gendep-impl,$(1),$(if $(2),$(2),$(call toupper,$(1)))))
define gendep-impl
  WITH_$(2) := $$(strip $$(WITH_$(2)))
  ifneq ($$(filter-out 0,$$(WITH_$(2))),)
    ifneq ($$($(2)_FLAGS)$$($(2)_LIBS),)
      HAVE_$(2) := 1
    endif
    ifneq ($$(HAVE_$(2)),) # all set
    else ifneq ($$(strip $$(filter pkg-config 1,$$(WITH_$(2)))),) # pkg-config
      ifneq ($$(call tool-exists,pkg-config),1)
        $$(call wmsg,"pkg-config not found, required to find $(1)")
      endif
      HAVE_$(2) := $$(call pkg-config,exists,$(1))
      ifeq ($$(HAVE_$(2)),1)
        $(2)_FLAGS ?= $$(call pkg-config,cflags,$(1))
        $(2)_LIBS ?= $$(call pkg-config,libs,$(1))
      endif
    else ifeq ($$(WITH_$(2)),cmake) # cmake
      ifneq ($$(call tool-exists,cmake),1)
        $$(call wmsg,"cmake not found, required to find $(1)")
      endif
      HAVE_$(2) := $$(call cmake-find-pkg,C,EXIST,$(1))
      ifeq ($$(HAVE_$(2)),1)
        $(2)_FLAGS ?= $$(call cmake-find-pkg,C,COMPILE,$(1))
        $(2)_LIBS ?= $$(call cmake-find-pkg,C,LINK,$(1))
      endif
    else # expect it in dir $(WITH_FOO)
      HAVE_$(2) := $$(shell [ -d $$(WITH_$(2)) ] && echo 1)
      ifeq ($$(HAVE_$(2)),1)
        $(2)_BASE ?= $$(call abspath,$$(WITH_$(2)))
        $(2)_FLAGS ?= -isystem $$($(2)_BASE)
        $(2)_LIBS ?= -L$$($(2)_BASE)/$(1) -Wl,-rpath,$$($(2)_BASE)/$(1) -l$(1)
      endif
    endif
    ifeq ($$(HAVE_$(2)),1)
      $(2)_PKG ?= $(1)
      HAVE_LIBS += -DHAVE_$(2)=1
      CFLAGS += $$($(2)_FLAGS)
      CXXFLAGS += $$($(2)_FLAGS)
      LDLIBS += $$($(2)_LIBS)
    else
      $$(call wmsg,"WITH_$(2)=$$(WITH_$(2)) but $(1) not found")
    endif
  endif # WITH_$(2) not empty
endef

# add all dependencies in $(GEN_DEPS) from makefile.package
$(foreach D,$(GEN_DEPS),$(call gendep,$(D)))

# add openni as a common dependency
$(call gendep,openni2-dev,OPENNI2)

# add vtk as a common dependency, find with cmake by default
WITH_VTK := $(strip $(if $(filter 1,$(WITH_VTK)),cmake,$(WITH_VTK)))
$(call gendep,VTK)

# add boost as a common dependency
# need to do this directly
# https://svn.boost.org/trac/boost/ticket/1094
WITH_BOOST := $(strip $(WITH_BOOST))
ifeq ($(WITH_BOOST),1)
  HAVE_BOOST := 1
  HAVE_LIBS += -DHAVE_BOOST=$(HAVE_BOOST)
  BOOST_LIBS ?= $(DEF_BOOST_LIBS) $(EXTRA_BOOST_LIBS)
  ifeq ($(IS_DARWIN),1)
    BOOST_LIBS := $(filter-out -lboost_mpi-mt,$(BOOST_LIBS))
  endif
  LDLIBS += $(BOOST_LIBS)
endif

# add cuda as a common dependency
WITH_CUDA := $(strip $(WITH_CUDA))
ifeq ($(WITH_CUDA),1)
  NVCC ?= $(shell which nvcc)
  HAVE_CUDA := $(if $(NVCC),$(shell $(NVCC) --version \
                                    > /dev/null 2>&1 && echo 1))
  ifneq ($(HAVE_CUDA),1)
    $(error "WITH_CUDA set but nvcc not found")
  endif
  HAVE_LIBS += -DHAVE_CUDA=$(HAVE_CUDA)
  CUDA_VERSION ?= $(shell $(NVCC) --version | grep release | \
                    tr ',' ' ' | cut -d' ' -f 6)
  DEF_CUDA_BASE := $(shell dirname $(shell dirname $(NVCC)))
  CUDA_BASE ?= $(DEF_CUDA_BASE)
  CUDA_LIB ?= $(CUDA_BASE)/lib$(if $(IS_LINUX),$(if $(IS_X86_64),64))
  CUDA_INC ?= $(CUDA_BASE)/include
  CUDA_LIBS ?= -lcudart
  LDFLAGS += -L$(CUDA_LIB) -Wl,-rpath,$(CUDA_LIB)
  LDLIBS += $(CUDA_LIBS)
  CFLAGS += -isystem $(CUDA_INC)
  CXXFLAGS += -isystem $(CUDA_INC)
  NVCCFLAGS += -isystem $(CUDA_INC)
  PCL_NVFLAGS += -D__CUDACC__ -DNVCC
  PCL_NVFLAGS += -m64
  PCL_NVFLAGS += -DEIGEN_USE_NEW_STDVECTOR \
                 -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
  PCL_NVFLAGS += -Dqh_QHpointer
  PCL_NVFLAGS += -Xcompiler -O3
  PCL_NVFLAGS += -Xcompiler -DNDEBUG
  PCL_NVFLAGS += -Xcompiler -fPIC
# unfortunately the inc-deps macro barfs if NVCCFLAGS has embedded commas
# also while there can be multiple -gencode there can be only one -arch option
#  PCL_NVFLAGS += -gencode arch=compute_20,code=sm_20
#  PCL_NVFLAGS += -gencode arch=compute_20,code=sm_21
#  PCL_NVFLAGS += -gencode arch=compute_30,code=sm_30
  PCL_NVFLAGS += -arch=compute_20
  PCL_NVFLAGS += --ftz=true --prec-div=false --prec-sqrt=false
  ifeq ($(IS_DARWIN),1)
    PCL_NVFLAGS += -Xcompiler -ftemplate-depth=1024
    PCL_NVFLAGS += -Xcompiler -Wno-invalid-offsetof
    PCL_NVFLAGS += -ccbin /usr/bin/clang -Xcompiler -Qunused-arguments
    ifeq ($(CC_IS_CLANG),1)
      ifeq ($(CUDA_VERSION),5.5)
        LDLIBS_LAST += /usr/lib/libstdc++.dylib
      endif
    endif
  endif
  NVCCFLAGS += $(PCL_NVFLAGS) $(HAVE_LIBS)
endif

# compile and link flags used by pcl
# reverse engineered by running make VERBOSE=1 during a pcl release build
PCL_FLAGS += -DEIGEN_USE_NEW_STDVECTOR \
             -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
PCL_FLAGS += -DNDEBUG -DEIGEN_NO_DEBUG -DQT_NO_DEBUG -DBOOST_DISABLE_ASSERTS 
PCL_FLAGS += -Dqh_QHpointer
PCL_FLAGS += -DQT_GUI_LIB -DQT_CORE_LIB
PCL_FLAGS += -Wall -Wextra -Wno-unknown-pragmas -Wno-format-extra-args \
             -Wno-sign-compare -Wno-invalid-offsetof -Wno-conversion \
             -Wabi -Wno-deprecated -Wno-unused-parameter
PCL_FLAGS += -fno-strict-aliasing
PCL_FLAGS += -fPIC
PCL_FLAGS += -march=native -msse4.2 -mfpmath=sse -fopenmp
PCL_FLAGS += -pthread
PCL_FLAGS += -O3

CFLAGS += $(filter-out -Wno-invalid-offsetof,$(PCL_FLAGS)) $(HAVE_LIBS)

CXXFLAGS += -ftemplate-depth=1024
CXXFLAGS += $(PCL_FLAGS) $(HAVE_LIBS)

CFLAGS += -DIN_$(call toupper,$(PROJECT))=1
CXXFLAGS += -DIN_$(call toupper,$(PROJECT))=1

#LDFLAGS += -Wl,--as-needed -rdynamic
LDFLAGS += -Wl,--no-as-needed -rdynamic

LDFLAGS += -fopenmp -Wl,-rpath,/usr/lib/openmpi/lib

fix-cflags-for-clang = $(patsubst -std=%++0x,-std=c++11,\
                        $(patsubst -fmax-errors=%,-ferror-limit=%,\
                          $(filter-out -mfpmath=% -fopenmp,$(1))))

ifeq ($(CC_IS_CLANG),1)
  CFLAGS := $(call fix-cflags-for-clang,$(CFLAGS))
  CFLAGS += -Qunused-arguments
  LDFLAGS := $(filter-out %as-needed -rdynamic,$(LDFLAGS))
  LDFLAGS := $(filter-out -fopenmp %openmpi/lib,$(LDFLAGS))
endif

ifeq ($(CXX_IS_CLANG),1)
  CXXFLAGS := $(call fix-cflags-for-clang,$(CXXFLAGS))
  CXXFLAGS += -Qunused-arguments
endif

ifeq ($(IS_DARWIN),1)
  LDFLAGS_SHARED += -dynamiclib -current_version 1.0.0 -undefined dynamic_lookup
  ifeq ($(CXX_IS_CLANG),1)
    CXXFLAGS := $(patsubst -std=c++11,-std=c++11 -stdlib=libc++,$(CXXFLAGS))
  endif
  ifeq ($(CC_IS_CLANG),1)
    LDLIBS += -lc++
  else
    LDFLAGS := $(filter-out %as-needed -rdynamic,$(LDFLAGS))
    LDLIBS += -lstdc++
  endif
else # not OS X
  LDFLAGS_SHARED += -shared
  LDLIBS += -lpthread -lstdc++
endif

#these may be needed too
#LDLIBS += -lpthread -lpng -lusb-1.0 -lm -ldl -lGLU -lGL -lSM -lICE -lX11 -lXext

# add link-last libs
LDLIBS += $(LDLIBS_LAST)

# get pcl modules a target depends on
# this is the value of the variable target_name_pcl_modules
# with . in the target name translated to _
# plus PCL_MODULES
# generally target_name_pcl_modules is set in makefile.project
# if unset then the target depends only on PCL_MODULES
mods = $(if $(WITH_PCL),$($(subst .,_,$(1))_pcl_modules) $(PCL_MODULES))

# get the compile flags for a target
# this is the value of the variable target_name_flags
# with . in the target name translated to _
# plus the flags for the pcl modules the target depends on
# generally target_name_flags is set in makefile.project
# if unset then the target flags are just the flags of the depended pcl modules
flags = $($(subst .,_,$(1))_flags) $(call pcl-cflags,$(call mods,$(1)))

# like flags but gets the link dependencies for a target
deps = $($(subst .,_,$(1))_deps) $(call pcl-libs,$(call mods,$(1)))

# get actual dependencies for a target
# the idea here is to translate foo.cpp to foo.o, etc
# because foo.o (generated from foo.cpp) is what the build actually depends on
# also if target is foo.bin and foo.c* exists then foo.o is an actual dep
actual-deps = $(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(patsubst %.cu,%.o, \
                $(wildcard $(if $(filter %.bin,$(1)),$(1:.bin=.c)*)) \
                $(call deps,$(1)))))

# get only the binary actual dependencies for a target
# filters out header files
bin-deps = $(filter-out %.h %.hpp,$(call actual-deps,$(1)))

# get only the file dependencies for a target
# keeps ony object and header files
# (the dependencies can include libraries too)
file-deps = $(filter %.o %.h %.hpp,$(call actual-deps,$(1)))

# get the included headers of a source file
# -MM skips system headers
# and we use -isystem instead of -I to mark system headers
# cached because it's slow, run make cacheclean or make realclean to reset
# arg1 is the compiler command
# arg2 are the compiler flags
# arg3 is the source filename, its extension is ignored
# arg4 is the extension to pretend the filename had
inc-deps = $(call inc-deps-cached,$(1),$(2),\
  $(basename $(3)).$(4),$(basename $(3)).o,.$(basename $(3)).$(4).deps)
inc-deps-cached = $(filter-out \ $(3) $(4):,\
$(shell [ ! -f $(3) ] || ( [ $(5) -nt $(3) ] && cat $(5) ) || \
  ( $(1) $(CPPFLAGS) $(call isystem-nvcc,$(1),$(2) $(call flags,$(4))) $(3) \
         $(if $(filter %nvcc,$(1)),\
              -Xcompiler -MM -Xcompiler -MF -Xcompiler $(5) -E > /dev/null,\
              -MM -MF $(5)) \
         && cat $(5) ) ))
isystem-nvcc = $(if $(filter %nvcc,$(1)),\
                    $(subst -isystem,-Xcompiler -isystem -Xcompiler,$(2)),\
                    $(2))

# emit a compile or link message
# arg1 is the target
# arg2 is the source
compile-msg = @echo -e $(call vt100,grn,compiling $(1) from $(2))
link-msg = @echo -e $(call vt100,blu,linking $(1) from $(2))

# generate a compile command
# arg1 is the compiler command
# arg2 are the compiler flags
# arg3 is the target filename
# arg4 is the source filename
compile-cmd = $(1) $(CPPFLAGS) $(2) $(call flags,$(3)) -c $(4)

# enable automatic variables in dependency lists
# $$ is needed for secondexpansion
.SECONDEXPANSION:

# keep intermediate object files
.PRECIOUS: %.o

# to show commands use
# make VERBOSE=1 ...
ifeq ("$(VERBOSE)","1")
Q :=
else
Q := @
endif

# build targets set in makefile.project
.PHONY: all bins libs docs
all: $(TARGETS)
bins: $(BIN_TARGETS)
libs: $(LIB_TARGETS)
docs: $(DOC_TARGETS)

# use this target as a prereq to force another target
.PHONY: FORCE
FORCE:

# pattern rules for compiling and linking
# cmd lines have same variables in the same order as the built-in implicit rules
# but some fanciness is added for massaging dependencies

# %.bin actually links % without the .bin extension
%.bin: $$(call file-deps,$$@)
	$(call link-msg,$(basename $@),$^)
	$(Q)$(CXX) $(LDFLAGS) -o $(basename $(@)) $(call bin-deps,$(@)) $(LOADLIBES) $(LDLIBS)

# %.lib actually links lib%.a and lib%.$(SOEXT)
%.lib: $$(call file-deps,$$@)
	$(call link-msg,lib$(basename $@).a,$^)
	$(Q)ar -cq  lib$(basename $@).a $^
	$(call link-msg,lib$(basename $@).$(SOEXT),$^)
	$(Q)$(CC) $(LDFLAGS_SHARED) \
	$(if $(IS_DARWIN),-install_name @rpath/lib$(basename $@).$(SOEXT)) \
	-o lib$(basename $@).$(SOEXT) $^

%.o: %.c $$(call file-deps,$$@) $$(call inc-deps,$(CC),$(CFLAGS),$$@,c)
	$(call compile-msg,$@,$^)
	$(Q)$(call compile-cmd,$(CC),$(CFLAGS),$@,$<)

%.o: %.cpp $$(call file-deps,$$@) $$(call inc-deps,$(CXX),$(CXXFLAGS),$$@,cpp)
	$(call compile-msg,$@,$^)
	$(Q)$(call compile-cmd,$(CXX),$(CXXFLAGS),$@,$<)

%.o: %.cu $$(call file-deps,$$@) $$(call inc-deps,$(NVCC),$(NVCCFLAGS),$$@,cu)
	$(call compile-msg,$@,$^)
	$(Q)$(call compile-cmd,$(NVCC),$(NVCCFLAGS),$@,$<)

#make foo.show-inc-deps
%.show-inc-deps: %.c 
	@echo $(call inc-deps,$(CC),$(CFLAGS),$@),c)

%.show-inc-deps: %.cpp 
	@echo $(call inc-deps,$(CXX),$(CXXFLAGS),$@,cpp)

%.show-inc-deps: %.cu
	@echo $(call inc-deps,$(NVCC),$(NVCCFLAGS),$@,cu)

#make foo.o.show
#make foo.bin.show
%.show:
	@echo $(basename $@) flags: $(call flags,$(basename $@))
	@echo $(basename $@) deps: $(call deps,$(basename $@))
	@echo $(basename $@) pcl flags: $(call pcl-cflags,$(call mods,$(basename $@)))
	@echo $(basename $@) pcl libs: $(call pcl-libs,$(call mods,$(basename $@)))
	@echo $(basename $@) pcl modules: $(call mods,$(basename $@))
	@echo $(basename $@) obj deps: $(call actual-deps,$(basename $@))
	@echo $(basename $@) bin deps: $(call bin-deps,$(basename $@))
	@echo $(basename $@) file deps: $(call file-deps,$(basename $@))
	@[ ! -e $(basename $(basename $@)).c ] || \
	echo $(basename $@) include deps: $(call inc-deps,$(CC),$(CFLAGS),$(basename $@),c)
	@[ ! -e $(basename $(basename $@)).cpp ] || \
	echo $(basename $@) include deps: $(call inc-deps,$(CXX),$(CXXFLAGS),$(basename $@),cpp)
	@[ ! -e $(basename $(basename $@)).cu ] || \
	echo $(basename $@) include deps: $(call inc-deps,$(NVCC),$(NVCCFLAGS),$(basename $@),cu)

.PHONY: show-settings
show-settings:
	@echo IS_DARWIN=$(IS_DARWIN) CC_IS_CLANG=$(CC_IS_CLANG) CXX_IS_CLANG=$(CXX_IS_CLANG)
	@echo GEN_DEPS=$(GEN_DEPS)
	@echo WITH_PCL=$(WITH_PCL) WITH_OPENNI=$(WITH_OPENNI) WITH_VTK=$(WITH_VTK) WITH_BOOST=$(WITH_BOOST) WITH_CUDA=$(WITH_CUDA) $(foreach D,$(call toupper,$(GEN_DEPS)),WITH_$(D)=$(WITH_$(D)) HAVE_$(D)=$(HAVE_$(D)) $(D)_PKG=$($(D)_PKG) $(D)_FLAGS=$($(D)_FLAGS) $(D)_LIBS=$($(D)_LIBS))
	@echo HAVE_LIBS=$(HAVE_LIBS)
	@echo PROJECT=$(PROJECT)
	@echo TARGETS=$(TARGETS)
	@echo PCL_VERSION=$(PCL_VERSION)
	@echo PCL_MODULES=$(PCL_MODULES)
	@echo $(PROJECT)_lib_deps=$($(PROJECT)_lib_deps)
	@echo SOEXT=$(SOEXT)
	@echo CFLAGS=$(CFLAGS)
	@echo CXXFLAGS=$(CXXFLAGS)
	@echo LDFLAGS=$(LDFLAGS)
	@echo LDFLAGS_SHARED=$(LDFLAGS_SHARED)
	@echo LDLIBS=$(LDLIBS)
ifeq ($(HAVE_BOOST),1)
	@echo BOOST_LIBS=$(BOOST_LIBS)
endif
ifeq ($(HAVE_CUDA),1)
	@echo CUDA_VERSION=$(CUDA_VERSION)
	@echo NVCCFLAGS=$(NVCCFLAGS)
endif
	@echo ZIP_EXCLUDE=$(ZIP_EXCLUDE)

.PHONY: clean binclean docclean cacheclean realclean

clean: $(EXTRA_CLEAN_DEPS)
	rm -f *.o

binclean:
	rm -f $(basename $(filter %.bin,$(TARGETS)))
	rm -f $(addprefix lib,$(subst .lib,.a,$(filter %.lib,$(TARGETS))))
	rm -f $(addprefix lib,$(subst .lib,.$(SOEXT),$(filter %.lib,$(TARGETS))))

docclean:
	rm -f $(addsuffix .html,$(basename $(wildcard *.m4d)))

cacheclean:
	rm -f .*.cache
	rm -f .*.deps

realclean: clean binclean docclean cacheclean $(EXTRA_REALCLEAN_DEPS)

# HTML stuff

WWW_REL_HOME ?= .
WWW_INC ?= $(WWW_REL_HOME)
M4_MACROS ?= $(WWW_INC)/macros.m4
CSS ?= $(WWW_INC)/style.css
HTML_M4_EXTRA_DEPS += $(WWW_INC)/*.m4
HTML_GEN_INC += "-I$(WWW_INC)" -DWWW_INC="$(WWW_INC)" $(M4_MACROS)
HTML_GEN_DEFS += -DWWW_REL_HOME="$(WWW_REL_HOME)" \
                 -DBUILD_DATE="`date`" -DYEAR="`date +%Y`" -DCSS="$(CSS)"
PANDOC_HTML_OPTS += --standalone --smart --css $(CSS)

%.html: %.md
	$(call compile-msg,$@,$^)
	$(Q) pandoc $(PANDOC_HTML_OPTS) -o $@ $<

%.html: %.m4d $(HTML_M4_EXTRA_DEPS)
	$(call compile-msg,$@,$<)
	$(Q) m4 -P $(HTML_GEN_DEFS) $(HTML_GEN_INC) $(HTML_M4_EXTRA) $< | \
	pandoc $(PANDOC_HTML_OPTS) -o $@

# package and publish stuff

RSYNC_FLAGS := -rv --progress \
               --exclude "**~" --exclude ".\#**" \
               --exclude CVS --exclude .svn --cvs-exclude \
               --exclude "*.tmp" --exclude "*.ORIG" --exclude "*.OLD" \
               --exclude "*.cache" --exclude "*.deps"

SVNVERSION_FILE := SVNVERSION.txt
SVNVERSION_STRING := $(strip \
  $(if $(wildcard $(SVNVERSION_FILE)),\
       R$(subst $(SPACE),_,$(filter-out %die,$(subst :,die ,\
        $(shell cat $(SVNVERSION_FILE))))),\
    unknown))

ZIPBN ?= $(PROJECT)-SVNVERSION
ZIPBN_NEWEST := $(subst SVNVERSION,newest,$(ZIPBN))
ZIPBN_PAT := $(subst SVNVERSION,R*,$(ZIPBN))
ZIPBN_SUBST := $(subst SVNVERSION,$(SVNVERSION_STRING),$(ZIPBN))
ZIP_DIR := .$(ZIPBN).tmp

$(SVNVERSION_FILE): FORCE
	svnversion > $@
	date +%Y-%m-%d >> $@

.PHONY: zip zip-impl
zip: $(SVNVERSION_FILE)
	$(MAKE) zip-impl
zip-impl: zipclean $(EXTRA_ZIP_DEPS)
	$(Q) mkdir $(ZIP_DIR)
	$(Q) mkdir $(ZIP_DIR)/$(ZIPBN_SUBST)
	$(Q) rsync $(RSYNC_FLAGS) --exclude "*.zip" \
		$(foreach f,$(ZIP_INCLUDE),--include $(f)) \
		$(foreach f,$(ZIP_EXCLUDE),--exclude $(f)) \
		* $(ZIP_DIR)/$(ZIPBN_SUBST)
	$(Q) cd $(ZIP_DIR) && zip -rp $(ZIPBN_SUBST).zip $(ZIPBN_SUBST)
	$(Q) mv $(ZIP_DIR)/$(ZIPBN_SUBST).zip .
	$(Q) ln -s $(ZIPBN_SUBST).zip $(ZIPBN_NEWEST).zip
	$(Q) $(RM) -rf $(ZIP_DIR)

$(ZIPBN_SUBST).zip: zip

.PHONY: svnversionclean zipclean

svnversionclean:
	rm -f $(SVNVERSION_FILE)

zipclean:
	rm -rf $(ZIP_DIR)
	rm -f $(ZIPBN_PAT).zip $(ZIPBN_NEWEST).zip

realclean: svnversionclean zipclean

.PHONY: publish publish-impl
publish: $(SVNVERSION_FILE)
	$(MAKE) publish-impl
publish-impl: $(EXTRA_PUBLISH_DEPS)
	@echo publishing to $(PUBLISH_DEST)
	$(Q) rsync --progress $(PUBLISH_DOCS) $(PUBLISH_DEST)
	$(Q) if [ -f $(ZIPBN_SUBST).zip ]; then \
	rsync --progress $(ZIPBN_SUBST).zip $(PUBLISH_DEST); \
	rsync --progress --links $(ZIPBN_NEWEST).zip $(PUBLISH_DEST); \
	else echo "$(ZIPBN_SUBST).zip not found, not publishing zip"; fi
	$(EXTRA_PUBLISH_CMDS)


