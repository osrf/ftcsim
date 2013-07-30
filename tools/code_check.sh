#!/bin/sh

# Jenkins will pass -xml, in which case we want to generate XML output
xmlout=0
if test "$1" = "-xmldir" -a -n "$2"; then
  xmlout=1
  xmldir=$2
  mkdir -p $xmldir
  rm -rf $xmldir/*.xml
  # Assuming that Jenkins called, the `build` directory is a sibling to the src dir
  builddir=../build
else
  # This is a heuristic guess; not every developer puts the `build` dir in the src dir
  builddir=./build
fi


#cppcheck
if [ $xmlout -eq 1 ]; then
  # Run most of the checks in parallel
  (cppcheck --xml --enable=style,performance,portability,information -j 4 -q `find ./ros -name "*.cc"`) 2> $xmldir/cppcheck.xml 

  # Unused function checking must happen in one job
  (cppcheck --xml --enable=unusedFunction -q `find ./ros -name "*.cc"`) 2> $xmldir/cppcheck-unused-functions.xml 

  # Finally, check the configuration
  (cppcheck --xml --enable=missingInclude -q -j 4 `find ./ros -name "*.cc"`  -I . --check-config) 2> $xmldir/cppcheck-configuration.xml
else
  # Run most of the checks in parallel
  cppcheck --enable=style,performance,portability,information -j 4 -q `find ./ros -name "*.cc"` 2>&1

  # Unused function checking must happen in one job
  cppcheck --enable=unusedFunction -q `find ./ros -name "*.cc"` 2>&1

  # Finally, check the configuration
  cppcheck --enable=missingInclude -q -j 4 `find ./ros -name "*.cc"` -I . --check-config 2>&1
fi

# cpplint
if [ $xmlout -eq 1 ]; then
  (find ./ros -print0 -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" | xargs -0 python tools/cpplint.py 2>&1) | python tools/cpplint_to_cppcheckxml.py 2> $xmldir/cpplint.xml
else
  find ./ros -print0 -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" | xargs -0 python tools/cpplint.py 2>&1
fi
