#include "mesh.h"


Vertex::Vertex(glm::vec4 pos, glm::vec4 nor, glm::vec2 uv, int objID, int faceID, glm::vec4 faceNor)
    :pos(pos), nor(nor), uv(uv), objID(objID), faceID(faceID), faceNor(faceNor)
{}

OCCEdge::OCCEdge(int id, TopoDS_Edge edge, int faceID) 
	:id(id), edge(edge), faceID(faceID)
{}

void OCCCompound::read(const std::string& filename) {
    const char* filenameTmp = filename.c_str();
    STEPControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile(filenameTmp);

    if (status != IFSelect_RetDone) {
        throw std::runtime_error("Error: Unable to read STEP file.");
    }

    reader.TransferRoots();
    shape = reader.OneShape();

    // 得到 STEP → TopoDS 的映射
    process = reader.WS()->TransferReader()->TransientProcess();

    // 这个方法不行，只能拿到不区分orientation的所有TopoDS_Edge
    /*int id = 0;
    TopTools_IndexedMapOfShape edgeMap;
    TopExp::MapShapes(shape, TopAbs_EDGE, edgeMap);
    for (int i = 1; i <= edgeMap.Extent(); ++i) {
        TopoDS_Edge e = TopoDS::Edge(edgeMap(i));
        edges.push_back(OCCEdge(id++, e));
    }*/


    TopTools_IndexedMapOfShape baseEdgeMap; //不区分orientation的edgeMap
    TopExp::MapShapes(shape, TopAbs_EDGE, baseEdgeMap);
    baseEdges.assign(baseEdgeMap.Extent(), {});

    // 这样才能拿到所有区分orientation的所有TopoDS_Edge
    {
        int edgeID = 0;
        int faceID = 0;
        faceEdgeCnt.push_back(0);
        for (TopExp_Explorer fExp(shape, TopAbs_FACE); fExp.More(); fExp.Next())
        {
            const TopoDS_Face& face = TopoDS::Face(fExp.Current());

            // 在当前 face 上遍历所有 edge
            for (TopExp_Explorer eExp(face, TopAbs_EDGE); eExp.More(); eExp.Next())
            {
                TopoDS_Edge edge = TopoDS::Edge(eExp.Current());
                edges.push_back(OCCEdge(edgeID, edge, faceID));
                int baseIndex = baseEdgeMap.FindIndex(edge);
                //OCC中baseIndex是从1开始的，所以这里我们减1
                --baseIndex;
                baseEdges[baseIndex].push_back(edgeID);
                ++edgeID;
            }
            faceEdgeCnt.push_back(edgeID);
            faces.push_back(face);
            ++faceID;
        }
        for (auto& be : baseEdges) {
            if (be.size() == 2) {
                edges[be[0]].sym = be[1];
                edges[be[1]].sym = be[0];
            }
        }
    }


    BRepMesh_IncrementalMesh mesher(shape, deflection, Standard_True, angle);

    //helper function。用于从重复的he中选出一条he进行渲染。
    //[&]代表对于没有写在参数里的变量都是引用拿来
    //如果变量写在参数里，像这里的heToEdgeID和symHE，则都是值传递。所以heToEdgeID和symHE在参数中需要改为引用
    auto chooseHe = [&](int n1, int n2, std::unordered_map<std::string, int>& boundaryHEIdx,
        std::unordered_map<std::string, int>& symHE, glm::vec4& normal, std::vector<Vertex>& vertices,
        std::vector<uint32_t>& indices) -> bool {
        //查看he是否是边界边
        auto it = boundaryHEIdx.find(std::to_string(n1) + "#" + std::to_string(n2));
        if (it != boundaryHEIdx.end()) {
            if (it->second == -1) {
                //这是不需要渲染的边界边
            }
            else {
                //需要渲染的边界边，但是已经加载过了，我们只把faceNor加给它
                mesh.vertices2[it->second].faceNor = normal;
                mesh.vertices2[it->second + 1].faceNor = normal;
            }
            //边界边提前加载过了，直接跳过
            return false;
        }
        //查看he的sym是否已经载入
        auto it2 = symHE.find(std::to_string(n2) + "#" + std::to_string(n1));
        if (it2 != symHE.end()) {
            vertices[it2->second].symFaceNor = normal;
            vertices[it2->second + 1].symFaceNor = normal;
            return false;
        }
        else {
            symHE[std::to_string(n1) + "#" + std::to_string(n2)] = indices.size();
            return true;
        }
        return false;
    };

    {
        for (int faceID = 0; faceID < faces.size(); ++faceID) {
            TopLoc_Location loc;
            TopoDS_Face& face = faces[faceID];

            Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
            if (tri.IsNull()) continue;

            const Poly_Array1OfTriangle& triangles = tri->Triangles();

            std::vector<glm::vec4> positions;
            //std::vector<glm::vec4> normals;
            std::vector<glm::vec2> uvs;
            for (int i = 1; i <= tri->NbNodes(); ++i) {
                positions.push_back(glm::vec4(tri->Node(i).X(), tri->Node(i).Y(), tri->Node(i).Z(), 1));
                /*normals.push_back(glm::vec4(0));
                if (tri->HasNormals()) {
                    normals.push_back(glm::vec4(tri->Normal(i).X(), tri->Normal(i).Y(), tri->Normal(i).Z(), 1));
                }*/
                uvs.push_back(glm::vec2(tri->UVNode(i).X(), tri->UVNode(i).Y()));
            }
            for (auto& p : positions) {
                p = p * 0.01f;
            }

            //先把所有边界边塞进vertices2
            //记录该face的mesh中的边界边，以及在vertices2中的id
            std::unordered_map<std::string, int> boundaryHEIdx;
            for (int i = faceEdgeCnt[faceID]; i < faceEdgeCnt[faceID + 1]; ++i) {
                bool shouldLoad = true;
                //如果该边界edge有sym，渲染id小的那一个
                if (edges[i].sym != -1) {
                    if (edges[i].sym < i) {
                        shouldLoad = false;
                    }
                }
                Handle(Poly_PolygonOnTriangulation) poly =
                    BRep_Tool::PolygonOnTriangulation(edges[i].edge, tri, loc);
                if (poly.IsNull()) {
                    mesh.bEdgesSE.push_back(-1);
                    mesh.bEdgesSE.push_back(-1);
                    continue;
                }
                int currBEdgeSE1 = mesh.indices2.size();
                const TColStd_Array1OfInteger& polyNodes = poly->Nodes();
                bool needReverse = (face.Orientation() != edges[i].edge.Orientation());
                if (needReverse) {
                    for (int j = polyNodes.Upper(); j > polyNodes.Lower(); --j) {
                        int n1 = polyNodes(j);
                        int n2 = polyNodes(j - 1);
                        --n1;
                        --n2;

                        std::string key = std::to_string(n1) + "#" + std::to_string(n2);
                        if (shouldLoad) {
                            boundaryHEIdx[key] = mesh.indices2.size();
                            mesh.vertices2.push_back(Vertex(positions[n1], glm::vec4(0), uvs[n1], faceID, i - 1, glm::vec4(0)));
                            mesh.vertices2.push_back(Vertex(positions[n2], glm::vec4(0), uvs[n2], faceID, i - 1, glm::vec4(0)));
                            mesh.indices2.push_back(mesh.indices2.size());
                            mesh.indices2.push_back(mesh.indices2.size());
                        }
                        else {
                            boundaryHEIdx[key] = -1;
                        }
                    }
                }
                else {
                    for (int j = polyNodes.Lower(); j < polyNodes.Upper(); ++j)
                    {
                        int n1 = polyNodes(j);
                        int n2 = polyNodes(j + 1);
                        --n1;
                        --n2;

                        std::string key = std::to_string(n1) + "#" + std::to_string(n2);
                        if (shouldLoad) {
                            boundaryHEIdx[key] = mesh.indices2.size();
                            mesh.vertices2.push_back(Vertex(positions[n1], glm::vec4(0), uvs[n1], faceID, i - 1, glm::vec4(0)));
                            mesh.vertices2.push_back(Vertex(positions[n2], glm::vec4(0), uvs[n2], faceID, i - 1, glm::vec4(0)));
                            mesh.indices2.push_back(mesh.indices2.size());
                            mesh.indices2.push_back(mesh.indices2.size());
                        }
                        else {
                            boundaryHEIdx[key] = -1;
                        }
                    }
                }
                int currBEdgeSE2 = mesh.indices2.size();
                mesh.bEdgesSE.push_back(currBEdgeSE1);
                mesh.bEdgesSE.push_back(currBEdgeSE2);
            }

            //记录该face的mesh中的sym he
            //[n1#n2] : {vertices2中的id,该面normal}
            std::unordered_map<std::string, int> symHE;
            for (int i = triangles.Lower(); i <= triangles.Upper(); ++i) {
                int n1, n2, n3; //node indices
                triangles(i).Get(n1, n2, n3);
                --n1;
                --n2;
                --n3;
                glm::vec3 v1 = positions[n2] - positions[n1];
                glm::vec3 v2 = positions[n3] - positions[n1];
                glm::vec4 normal = glm::vec4(glm::normalize(glm::cross(v1, v2)), 0);
                mesh.vertices1.push_back(Vertex(positions[n1], normal, uvs[n1], faceID, i - 1, normal));
                mesh.vertices1.push_back(Vertex(positions[n2], normal, uvs[n2], faceID, i - 1, normal));
                mesh.vertices1.push_back(Vertex(positions[n3], normal, uvs[n3], faceID, i - 1, normal));
                mesh.indices1.push_back(mesh.indices1.size());
                mesh.indices1.push_back(mesh.indices1.size());
                mesh.indices1.push_back(mesh.indices1.size());
                if (chooseHe(n1, n2, boundaryHEIdx, symHE, normal, mesh.vertices2, mesh.indices2 )) {
                    mesh.vertices2.push_back(Vertex(positions[n1], normal, uvs[n1], faceID, i - 1, normal));
                    mesh.vertices2.push_back(Vertex(positions[n2], normal, uvs[n2], faceID, i - 1, normal));
                    mesh.indices2.push_back(mesh.indices2.size());
                    mesh.indices2.push_back(mesh.indices2.size());
                }
                if (chooseHe(n2, n3, boundaryHEIdx, symHE, normal, mesh.vertices2, mesh.indices2)) {
                    mesh.vertices2.push_back(Vertex(positions[n2], normal, uvs[n2], faceID, i - 1, normal));
                    mesh.vertices2.push_back(Vertex(positions[n3], normal, uvs[n3], faceID, i - 1, normal));
                    mesh.indices2.push_back(mesh.indices2.size());
                    mesh.indices2.push_back(mesh.indices2.size());
                }
                if (chooseHe(n3, n1, boundaryHEIdx, symHE, normal, mesh.vertices2, mesh.indices2)) {
                    mesh.vertices2.push_back(Vertex(positions[n3], normal, uvs[n3], faceID, i - 1, normal));
                    mesh.vertices2.push_back(Vertex(positions[n1], normal, uvs[n1], faceID, i - 1, normal));
                    mesh.indices2.push_back(mesh.indices2.size());
                    mesh.indices2.push_back(mesh.indices2.size());
                }

            }

        }

        
    }

    {
        int debug = 0;
        debug = debug;
        int visChange[10] = {1,1,1,1,1,1,1,1,1,1};
        int interCnt = 4;
        int vis[11];
        for (int i = 1; i < 11; ++i) {
            vis[i] = 0;
        }
        vis[0] = 0;
        for (int i = 0; i < interCnt; ++i) {
            vis[i + 1] = vis[i] + visChange[i];
        }
        //最高降低到0
        int maxVis = -11;
        for (int i = 0; i <= interCnt; ++i) {
            if (vis[i] > maxVis) {
                maxVis = vis[i];
            }
        }
        for (int i = 0; i <= interCnt; ++i) {
            vis[i] -= maxVis;
        }
        debug = debug;
    }
};