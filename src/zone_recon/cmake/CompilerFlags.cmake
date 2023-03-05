# Ensure our code is good by enabling lots of warnings
string(CONCAT GCC_COMPILER_WARNINGS " -Wpedantic"
                                    " -Wall"
                                    " -Wextra"
                                    " -Wno-system-headers"
                                    " -Wswitch-default"
                                    " -Wfloat-equal"
                                    " -Wshadow"
                                    " -Wcast-qual"
                                    " -Wconversion"
                                    " -Wsign-conversion"
                                    " -Wlogical-op"
                                    " -Wnon-virtual-dtor"
                                    " -Wold-style-cast"
                                    " -Woverloaded-virtual"
                                    " -Wuninitialized"
                                    " -Wsuggest-override"
                                    " -Wundef"
                                    " -Wcast-qual"
                                    " -Wcast-align"
                                    " -Wzero-as-null-pointer-constant"
                                    " -Wredundant-decls"
                                    " -Wreturn-type"
    )

string(CONCAT CLANG_COMPILER_WARNINGS " -Wpedantic"
                                      " -Wall"
                                      " -Wextra"
                                      " -Wabstract-vbase-init"
                                      " -Warray-bounds-pointer-arithmetic"
                                      " -Wassign-enum"
                                      " -Wbad-function-cast"
                                      " -Wbitfield-enum-conversion"
                                      " -Wc++11-narrowing"
                                      " -Wc++14-compat-pedantic"
                                      " -Wcast-align"
                                      " -Wcast-qual"
                                      " -Wcomma"
                                      " -Wcomment"
                                      " -Wconditional-type-mismatch"
                                      " -Wconditional-uninitialized"
                                      " -Wcstring-format-directive"
                                      " -Wcustom-atomic-properties"
                                      " -Wdeprecated-implementations"
                                      " -Wdeprecated-increment-bool"
                                      " -Wdouble-promotion"
                                      " -Weffc++"
                                      " -Wextra-semi"
                                      " -Wfloat-conversion"
                                      " -Wfloat-equal"
                                      " -Wloop-analysis"
                                      " -Wformat-pedantic"
                                      " -Wfour-char-constants"
                                      " -Wgcc-compat"
                                      " -Wheader-hygiene"
                                      " -Winfinite-recursion"
                                      " -Wmethod-signatures"
                                      " -Wmismatched-tags"
                                      " -Wmissing-braces"
                                      " -Wmissing-field-initializers"
                                      " -Wmost"
                                      " -Wmove")


# CLAG and GCC don't agree on all the warning names        
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # using Clang
    MESSAGE("Using clang")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${CLANG_COMPILER_WARNINGS}")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # using GCC
    MESSAGE("USING GCC")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COMPILER_WARNINGS} -fopenmp -ggdb -O3")

    IF(NOT CMAKE_CROSSCOMPILING)
        SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native")
    ENDIF()
else()
  ERROR("Could not detect compiler")
endif()

if(WARNINGS_AS_ERROR)
    MESSAGE("Treating warnings as errors")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Werror")
endif()
