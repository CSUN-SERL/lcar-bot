# CMake generated Testfile for 
# Source directory: /home/thomas/lcar-bot/src/vision_opencv/src/test
# Build directory: /home/thomas/lcar-bot/build/vision_opencv/src/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(_ctest_cv_bridge_gtest_cv_bridge-utest "/home/thomas/lcar-bot/build/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/indigo/share/catkin/cmake/test/run_tests.py" "/home/thomas/lcar-bot/build/test_results/cv_bridge/gtest-cv_bridge-utest.xml" "--return-code" "/home/thomas/lcar-bot/devel/lib/cv_bridge/cv_bridge-utest --gtest_output=xml:/home/thomas/lcar-bot/build/test_results/cv_bridge/gtest-cv_bridge-utest.xml")
ADD_TEST(_ctest_cv_bridge_nosetests_enumerants.py "/home/thomas/lcar-bot/build/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/indigo/share/catkin/cmake/test/run_tests.py" "/home/thomas/lcar-bot/build/test_results/cv_bridge/nosetests-enumerants.py.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/thomas/lcar-bot/build/test_results/cv_bridge" "/usr/bin/nosetests-2.7 -P --process-timeout=60 /home/thomas/lcar-bot/src/vision_opencv/src/test/enumerants.py --with-xunit --xunit-file=/home/thomas/lcar-bot/build/test_results/cv_bridge/nosetests-enumerants.py.xml")
