for dir in $(ls -d */)
do
    cd $dir
    categoryName=${dir%%/}
    headerFile=${categoryName}Factory.hpp
    sourceFile=${categoryName}Factory.cpp
    # Clean generated content
    rm -f $headerFile $sourceFile
    # Get all Filters
    filterNames=$(ls *.hpp | sed s/.hpp//g)

    # Write header
    echo "#pragma once" > $headerFile
    echo "namespace Vision {" >> $headerFile
    echo "  namespace Filters {" >> $headerFile
    echo "    void register${categoryName}Filters();" >>$headerFile
    echo "  }" >> $headerFile
    echo "}" >> $headerFile
    # Write source
    echo "#include \"${headerFile}\"" > $sourceFile
    echo "" >> $sourceFile
    # Include all filter header files
    for filter in $filterNames
    do
        echo "#include \"$filter.hpp\"" >> $sourceFile
    done
    # Include filterFactory
    echo "" >> $sourceFile
    echo "#include \"../FilterFactory.hpp\"" >> $sourceFile
    # Entering namespace
    echo "" >> $sourceFile
    echo "namespace Vision {" >> $sourceFile
    echo "  namespace Filters {" >> $sourceFile
    # Declare function
    echo "    void register${categoryName}Filters()" >> $sourceFile
    echo "    {" >> $sourceFile
    for filter in $filterNames
    do
        echo "      FilterFactory::registerClass<${filter}>(\"$filter\");" >> $sourceFile
    done
    echo "    }" >> $sourceFile
    # Closing namespaces
    echo "  }" >> $sourceFile
    echo "}" >> $sourceFile
    cd ..
done
