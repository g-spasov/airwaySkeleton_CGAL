#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>
#include <CGAL/boost/graph/split_graph_into_polylines.h>
#include <fstream>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Real_timer.h>
#include <CGAL/Vector_3.h>
#include <CGAL/IO/STL.h>
#include <CGAL/boost/graph/generators.h>
#include <CGAL/boost/graph/IO/STL.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <filesystem>
#include <CGAL/Aff_transformation_3.h>

typedef CGAL::Simple_cartesian<double>                        Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel2;
typedef Kernel::Point_3                                       Point;
typedef CGAL::Polyhedron_3<Kernel>                            Polyhedron;
typedef boost::graph_traits<Polyhedron>::vertex_descriptor    vertex_descriptor;
typedef CGAL::Mean_curvature_flow_skeletonization<Polyhedron> Skeletonization;
typedef Skeletonization::Skeleton                             Skeleton;
typedef Skeleton::vertex_descriptor                           Skeleton_vertex;
typedef Skeleton::edge_descriptor                             Skeleton_edge;
namespace PMP = CGAL::Polygon_mesh_processing;
typedef CGAL::Surface_mesh<Point>                              Surface_mesh;


//only needed for the display of the skeleton as maximal polylines
struct Display_polylines{
  const Skeleton& skeleton;
  std::ofstream& out;
  int polyline_size;
  std::stringstream sstr;
  Display_polylines(const Skeleton& skeleton, std::ofstream& out)
    : skeleton(skeleton), out(out)
  {}
  void start_new_polyline(){
    polyline_size=0;
    sstr.str("");
    sstr.clear();
  }
  void add_node(Skeleton_vertex v){
    ++polyline_size;
    sstr << " " << skeleton[v].point;
  }
  void end_polyline()
  {
    out << polyline_size << sstr.str() << "\n";
  }
};
// This example extracts a medially centered skeleton from a given mesh.
int main(int argc, char* argv[])
{
  if(argc <2){
    std::cout << "Usage: " << argv[0] << " [imput mesh file]" << std::endl;
    return EXIT_FAILURE;
  }
  const std::string filename =argv[1]; // (argc > 1) ? argv[1] : CGAL::data_file_path("meshes/pig.stl");
  const std::string saveFile = filename.substr(0,filename.find("."));

  Surface_mesh sm;
  std::cout << "Reading " << filename << std::endl;
  if(!PMP::IO::read_polygon_mesh(filename, sm) || sm.is_empty())
  {
    std::cerr << "Invalid input file." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Done reading the mesh" << std::endl;
  Polyhedron tmesh;

  std::cout << "Converting the mesh " << std::endl;
  CGAL::copy_face_graph(sm, tmesh);
  std::cout << "Done converting the mesh " << std::endl;


  Skeleton skeleton;
  CGAL::extract_mean_curvature_flow_skeleton(tmesh, skeleton);
  std::cout << "Number of vertices of the skeleton: " << boost::num_vertices(skeleton) << "\n";
  std::cout << "Number of edges of the skeleton: " << boost::num_edges(skeleton) << "\n";
  // Output all the edges of the skeleton.
  std::ofstream output("skel-poly.polylines.txt");
  Display_polylines display(skeleton,output);
  CGAL::split_graph_into_polylines(skeleton, display);
  output.close();
  // Output skeleton points and the corresponding surface points
  output.open("correspondance-poly.polylines.txt");
  for(Skeleton_vertex v : CGAL::make_range(vertices(skeleton)))
    for(vertex_descriptor vd : skeleton[v].vertices)
      output << "2 " << skeleton[v].point << " "
                     << get(CGAL::vertex_point, tmesh, vd)  << "\n";
  return EXIT_SUCCESS;
}
