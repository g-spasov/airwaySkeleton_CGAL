#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Mean_curvature_flow_skeletonization.h>
#include <fstream>
#include <CGAL/boost/graph/IO/STL.h>
#include <CGAL/IO/STL.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <filesystem>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>



typedef CGAL::Simple_cartesian<double>                        Kernel;
typedef Kernel::Point_3                                       Point;
typedef CGAL::Surface_mesh<Point>                             Triangle_mesh;
typedef boost::graph_traits<Triangle_mesh>::vertex_descriptor vertex_descriptor;
typedef CGAL::Mean_curvature_flow_skeletonization<Triangle_mesh> Skeletonization;
typedef Skeletonization::Skeleton                             Skeleton;
typedef Skeleton::vertex_descriptor                           Skeleton_vertex;
typedef Skeleton::edge_descriptor                             Skeleton_edge;

namespace PMP = CGAL::Polygon_mesh_processing;



int main(int argc, char* argv[])
{
  if(argc <2){
    std::cout << "Usage: " << argv[0] << " [imput mesh file]" << std::endl;
    return EXIT_FAILURE;
  }
  const std::string filename =argv[1]; // (argc > 1) ? argv[1] : CGAL::data_file_path("meshes/pig.stl");
  const std::string saveFile = filename.substr(0,filename.find("."));

  Triangle_mesh tmesh;
  std::cout << "Reading " << filename << std::endl;
  if(!PMP::IO::read_polygon_mesh(filename, tmesh) || tmesh.is_empty())
  {
    std::cerr << "Invalid input file." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Done reading the mesh" << std::endl;
  
  

  //   Triangle_mesh tmesh; 
  //   CGAL::copy_face_graph(sm, tmesh);
  double pi=3.14159265358979323846;
  Skeleton skeleton;
  Skeletonization mcs(tmesh);

  std::cout << mcs.max_triangle_angle() << std::endl;
  std::cout << mcs.min_edge_length() << std::endl;

  mcs.set_is_medially_centered(false);
  mcs.set_max_triangle_angle(150*pi/180);
  mcs.set_min_edge_length(0.1);


  std::cout << mcs.max_triangle_angle() << std::endl;
  std::cout << mcs.min_edge_length() << std::endl;
  // 1. Contract the mesh by mean curvature flow.
  mcs.contract_geometry();
  // 2. Collapse short edges and split bad triangles.
  mcs.collapse_edges();
  mcs.split_faces();
  // 3. Fix degenerate vertices.
  mcs.detect_degeneracies();
  // Perform the above three steps in one iteration.
  mcs.contract();
  // Iteratively apply step 1 to 3 until convergence.
  mcs.contract_until_convergence();
  // Convert the contracted mesh into a curve skeleton and
  // get the correspondent surface points
  mcs.convert_to_skeleton(skeleton);
  std::cout << "Number of vertices of the skeleton: " << boost::num_vertices(skeleton) << "\n";
  std::cout << "Number of edges of the skeleton: " << boost::num_edges(skeleton) << "\n";
  // Output all the edges of the skeleton.
  std::ofstream output("skel-sm.polylines.txt");
  for(Skeleton_edge e : CGAL::make_range(edges(skeleton)))
  {
    const Point& s = skeleton[source(e, skeleton)].point;
    const Point& t = skeleton[target(e, skeleton)].point;
    output << "2 "<< s << " " << t << "\n";
  }
  output.close();
  // Output skeleton points and the corresponding surface points
  output.open("correspondance-sm.polylines.txt");
  for(Skeleton_vertex v : CGAL::make_range(vertices(skeleton)))
    for(vertex_descriptor vd : skeleton[v].vertices)
      output << "2 " << skeleton[v].point << "  " << get(CGAL::vertex_point, tmesh, vd)  << "\n";
  return EXIT_SUCCESS;
}