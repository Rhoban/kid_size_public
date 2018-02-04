Filters
=======

Factory
-------
In order to allow dynamic creation of object from xml, Filter can be created
through the use of the `FilterFactory`. Since there is a large number of filters,
we decided to separate them in different categories.

In order to reduce the compilation time when the header of a filter is modified,
each category has its own factory. Thus, the global factory does not need to be
updated since it references the header of the category factory which is not
supposed to be modified. This modification has been brought because the
compilation of `FilterFactory` was starting to be particularly long, especially
while compiling on the robot.

Example: The filter `Goal/GoalByROI.hpp` is modified, then, only the two
following files need to be updated: `GoalByROI.cpp` and `GoalFactory.cpp`

Auto-update
-----------
Since maintaining the consistency between the filters, their factory and the
`Sources.cmake` can be quite time consuming, two scripts are provided:
`create_cmake.sh` which create/update the `Sources.cmake` of all sub-categories
and `create_sub_factories.sh` which create/update the factories of all
sub-categories.