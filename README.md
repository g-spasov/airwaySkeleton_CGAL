# airwaySkeleton_CGAL

This program uses the CGAL library to generate a skeleton.
    1.skeleton.cpp -> Creates a skeleton from the default mesh style in CGAL, quite similar to the tutorial
    2.sm-skeleton.cpp -> Creates a skeleton from a surface mesh (stl) and uses:
        a. mcs.set_is_medially_centered(false);
        b. mcs.set_max_triangle_angle(150*pi/180);
        c. mcs.set_min_edge_length(0.1); <- This is the most important ot obtain proper 

    ##To compile
    mkdir build
    cd build 
    cmake .. 
    cmake --build .

