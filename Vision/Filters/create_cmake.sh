for dir in $(ls -d */)
do
    cd $dir
    echo "set (SOURCES" > Sources.cmake
    for file in $(ls *.cpp)
    do
        echo "  $file" >> Sources.cmake
    done
    echo ")" >> Sources.cmake
    cd ..
done
