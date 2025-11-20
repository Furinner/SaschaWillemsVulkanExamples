#include <algorithm>
#include <vector>
#include <memory>
#include <limits>
#include <unordered_set>
#include <unordered_map>
#include <optional>
#include <STEPControl_Reader.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopLoc_Location.hxx>
#include <Geom_Surface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Line.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom_Curve.hxx>                 // 曲线基类
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2dAPI_InterCurveCurve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomAPI.hxx>  
#include <GeomLProp_SLProps.hxx>
#include <Geom2dLProp_CLProps2d.hxx>
#include <gp_Pnt.hxx>                     // 点类
#include <Standard_Real.hxx>             // 实数类型
#include <TColgp_Array1OfPnt.hxx>        // 点数组（可选）
#include <BRepLib.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRep_PolygonOnTriangulation.hxx>
#include <ShapeFix_Edge.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <HLRBRep_Algo.hxx>
#include <HLRAlgo_Projector.hxx>
#include <HLRBRep_HLRToShape.hxx>
#include <XSControl_WorkSession.hxx>
#include <Transfer_TransientProcess.hxx>
#include <Transfer_FinderProcess.hxx>
#include <XSControl_TransferReader.hxx>
#include <TopTools_IndexedDataMapOfShapeShape.hxx>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include "VulkanglTFModel.h"

class Vertex {
public:
	/*alignas(16) glm::vec4 pos;
	alignas(16) glm::vec4 nor;
	alignas(16) glm::vec4 faceNor = glm::vec4(0);
	alignas(16) glm::vec4 symFaceNor = glm::vec4(0);
	alignas(8) glm::vec2 uv;
	alignas(4) int objID;
	alignas(4) int faceID;*/
	alignas(16) glm::vec3 pos;
	alignas(16) glm::vec3 nor;
	alignas(16) glm::vec3 faceNor = glm::vec3(0);
	alignas(16) glm::vec3 symFaceNor = glm::vec3(0);
	alignas(8) glm::vec2 uv;
	alignas(4) int objID;
	alignas(4) int faceID;
	alignas(4) int border = 0;
	alignas(4) int heID = -1;
	alignas(4) int uniqueID; //unique id in obj
	alignas(4) int debug = 0;
	alignas(4) int globalHeID = -1;

	Vertex(glm::vec4 pos, glm::vec4 nor, glm::vec2 uv, int objID, int faceID, glm::vec4 faceNor);
};

class Mesh {
public:
	std::vector<Vertex> vertices1; //simple vertices used for rendering depth map
	std::vector<Vertex> vertices2; //vertices used for edge
	std::vector<uint32_t> indices1{}; //0,1,2,3,4,5,6... 与vertices1的size一样
	std::vector<uint32_t> indices2{}; //0,1,2,3,4...  与vertices2的size一样

	//boundary
	//start point and end point of every boundary edges in vertices2
	//2个值一组，如果该bEdge没有被加载，则2个值一样
	std::vector<int> bEdgesSE;
};


class OCCEdge {
public:
	int id;
	TopoDS_Edge edge;
	int sym = -1;
	int faceID;

public:
	OCCEdge(int id, TopoDS_Edge edge, int faceID);
};

class OCCCompound {
public:
	TopoDS_Shape shape;
	std::vector<OCCEdge> edges;  //带orientation的
	// baseEdgeID : {edgeID1, edgeID2}
	std::vector<std::vector<int>> baseEdges;  //不带orientation的
	// faceID : TopoDS_Face
	std::vector<TopoDS_Face> faces;
	// 长度为faces的长度+1
	// 0, face0 end edge id, face1 end edge id,...
	std::vector<int> faceEdgeCnt;
	
	double deflection = 0.1;
	double angle = 0.1; //0.2 radiance

	// 得到 STEP → TopoDS 的映射
	Handle(Transfer_TransientProcess) process;

	Mesh mesh;
	
	void read(const std::string& filename);

	//helper Function

};
