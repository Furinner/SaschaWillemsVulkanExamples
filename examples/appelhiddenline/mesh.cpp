#include "mesh.h"


Vertex::Vertex(glm::vec4 pos, glm::vec4 nor, glm::vec2 uv, int objID, int faceID, glm::vec4 faceNor, int uniqueID)
    :pos(pos), nor(nor), uv(uv), objID(objID), faceID(faceID), faceNor(faceNor), uniqueID(uniqueID)
{}

OCCEdge::OCCEdge(int id, TopoDS_Edge edge, int faceID) 
	:id(id), edge(edge), faceID(faceID)
{}

void OCCCompound::meshToObjFile(const std::string& filename) {
    std::ofstream obj(filename);
    if (!obj.is_open())
        return;

    std::vector<int> offsets = {0};

    // 遍历所有 face
    int faceID = 0;
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        TopoDS_Face face = TopoDS::Face(exp.Current());

        TopLoc_Location loc;
        Handle(Poly_Triangulation) triangulation =
            BRep_Tool::Triangulation(face, loc);

        if (triangulation.IsNull())
            continue;

        const Poly_Array1OfTriangle& triangles = triangulation->Triangles();

        // ---------- 写顶点 ----------
        for (int i = 1; i <= triangulation->NbNodes(); ++i) {
            gp_Pnt p = triangulation->Node(i);
            obj << "v "
                << p.X() << " "
                << p.Y() << " "
                << p.Z() << "\n";
        }

        // ---------- 写三角形 ----------
        for (int i = triangles.Lower(); i <= triangles.Upper(); ++i)
        {
            int n1, n2, n3;
            triangles(i).Get(n1, n2, n3);

            // 处理 face 方向
            if (face.Orientation() == TopAbs_REVERSED)
                std::swap(n2, n3);

            obj << "f "
                << n1 + offsets[faceID] << " "
                << n2 + offsets[faceID] << " "
                << n3 + offsets[faceID] << "\n";
        }
        ++faceID;
        offsets.push_back(offsets[faceID - 1] + triangulation->NbNodes());
    }

    obj.close();
}

void OCCCompound::findAllConnectedEdges(int edgeId, std::vector<std::vector<int>>& bEdgesIdx, std::vector<OCCEdge>& bOccEdges, 
    std::vector<int>& startConnected, std::vector<int>& startConnectedAway, 
    std::vector<int>& endConnected, std::vector<int>& endConnectedAway) {
    //start
    int prev = bOccEdges[edgeId].prev;
    int prevSym = bOccEdges[prev].sym;
    while (prevSym != edgeId) {
        if (prevSym == -1) {
            break;
        }
        if (bOccEdges[prev].render) {
            startConnected.push_back(bEdgesIdx[prev][bEdgesIdx[prev].size() - 1]);
            startConnectedAway.push_back(1);
        }
        else {
            startConnected.push_back(bEdgesIdx[prevSym][0]);
            startConnectedAway.push_back(0);
        }
		prev = bOccEdges[prevSym].prev;
		prevSym = bOccEdges[prev].sym;
    }
	//end
	int next = bOccEdges[edgeId].next;
	int nextSym = bOccEdges[next].sym;
    while (nextSym != edgeId) {
        if (nextSym == -1) {
            break;
        }
        if (bOccEdges[next].render) {
            endConnected.push_back(bEdgesIdx[next][0]);
            endConnectedAway.push_back(0);
        }
        else {
            endConnected.push_back(bEdgesIdx[nextSym][bEdgesIdx[nextSym].size() - 1]);
            endConnectedAway.push_back(1);
        }
        next = bOccEdges[nextSym].next;
        nextSym = bOccEdges[next].sym;
    }
    for (int i = startConnected.size(); i < MAX_BEDGE_NEIGHBORS; ++i) {
		startConnected.push_back(-1);
		startConnectedAway.push_back(-1);
	}
    for (int i = endConnected.size(); i < MAX_BEDGE_NEIGHBORS; ++i) {
        endConnected.push_back(-1);
        endConnectedAway.push_back(-1);
    }
}

static float cross2D(glm::vec2 a, glm::vec2 b)
{
    return a.x * b.y - a.y * b.x;
}

static bool segmentsIntersect(glm::vec2 p1, glm::vec2 p2,
    glm::vec2 q1, glm::vec2 q2)
{

    glm::vec2 r = p2 - p1;
    glm::vec2 s = q2 - q1;

    float a = glm::length(r);
    float b = glm::length(s);
    const float EPS = 1e-8 * (glm::length(r) * glm::length(s) + 1.0f);

    float rxs = cross2D(r, s);
    float q_p_x_r = cross2D(q1 - p1, r);

    // 默认输出先设成 0
    glm::vec2 intersection = glm::vec2(0.0);

    // 共线
    if (abs(rxs) < EPS && abs(q_p_x_r) < EPS)
    {
        return false;
        // 这里“交点”不唯一，可以根据需要返回某种代表值
        // 例如返回 q1，或者重叠区间中点，这里简单返回 q1
        intersection = q1;
        // 用上面同样的重叠判定
        float r2 = dot(r, r);
        if (r2 < EPS) {
            return length(q1 - p1) < EPS || length(q2 - p1) < EPS;
        }
        float t0 = dot(q1 - p1, r) / r2;
        float t1 = dot(q2 - p1, r) / r2;
        if (t0 > t1) {
            float tmp = t0;
            t0 = t1;
            t1 = tmp;
        }
        return t0 <= 1.0 + EPS && t1 >= 0.0 - EPS;
    }

    // 平行不共线
    if (abs(rxs) < EPS && abs(q_p_x_r) >= EPS)
    {
        return false;
    }

    // 一般情况
    float t = cross2D(q1 - p1, s) / rxs;
    float u = cross2D(q1 - p1, r) / rxs;

    if (t >= 0.0 && t <= 1.0 &&
        u >= 0.0 && u <= 1.0)
    {
        // 只要相交，就用 t 算出交点
        //intersection = p1 + t * r;
        intersection = glm::vec2(t, u);
        return true;
    }

    return false;
}

void OCCCompound::read(const std::string& filename) {


	//glm::vec2 mainPos1 = glm::vec2(-0.55444831f, 0.90622330f);
	//glm::vec2 mainPos2 = glm::vec2(-0.11338938f, 0.72328508f);
 //   glm::vec2 currPos1 = glm::vec2(-0.21734953f, 0.76640475f);
 //   glm::vec2 currPos2 = glm::vec2(-0.21732196f, 0.76641756f);
    glm::vec2 mainPos1 = glm::vec2(-0.55444831f, 0.90622330f);
    glm::vec2 mainPos2 = glm::vec2(-0.11338938f, 0.72328508f);
    glm::vec2 currPos1 = glm::vec2(-0.21737374f, 0.76642179f);
    glm::vec2 currPos2 = glm::vec2(-0.21734953f, 0.76640475f);
    segmentsIntersect(mainPos1, mainPos2, currPos1, currPos2);

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
    //{
    //    int edgeID = 0;
    //    int faceID = 0;
    //    faceEdgeCnt.push_back(0);
    //    for (TopExp_Explorer fExp(shape, TopAbs_FACE); fExp.More(); fExp.Next())
    //    {
    //        const TopoDS_Face& face = TopoDS::Face(fExp.Current());

    //        // 在当前 face 上遍历所有 edge
    //        for (TopExp_Explorer eExp(face, TopAbs_EDGE); eExp.More(); eExp.Next())
    //        {
    //            TopoDS_Edge edge = TopoDS::Edge(eExp.Current());
    //            bOccEdges.push_back(OCCEdge(edgeID, edge, faceID));
    //            int baseIndex = baseEdgeMap.FindIndex(edge);
    //            //OCC中baseIndex是从1开始的，所以这里我们减1
    //            --baseIndex;
    //            baseEdges[baseIndex].push_back(edgeID);
    //            ++edgeID;
    //        }
    //        faceEdgeCnt.push_back(edgeID);
    //        faces.push_back(face);
    //        ++faceID;
    //    }
    //    for (auto& be : baseEdges) {
    //        if (be.size() == 2) {
    //            bOccEdges[be[0]].sym = be[1];
    //            bOccEdges[be[0]].symFaceID = bOccEdges[be[1]].faceID;
    //            bOccEdges[be[1]].sym = be[0];
    //            bOccEdges[be[1]].symFaceID = bOccEdges[be[0]].faceID;
    //        }
    //    }
    //}

    // 这样才能拿到所有区分orientation的所有TopoDS_Edge
    {
        int edgeID = 0;
        int faceID = 0;
        faceEdgeCnt.push_back(0);
        for (TopExp_Explorer fExp(shape, TopAbs_FACE); fExp.More(); fExp.Next())
        {
            const TopoDS_Face& face = TopoDS::Face(fExp.Current());
            //遍历当前face的所有wire
            for (TopExp_Explorer wExp(face, TopAbs_WIRE); wExp.More(); wExp.Next())
            {
                TopoDS_Wire wire = TopoDS::Wire(wExp.Current());
				int wireStartEdgeID = edgeID;
                for (BRepTools_WireExplorer we(wire); we.More(); we.Next()) {

                    TopoDS_Edge edge = TopoDS::Edge(we.Current());
                    bOccEdges.push_back(OCCEdge(edgeID, edge, faceID));
                    int baseIndex = baseEdgeMap.FindIndex(edge);
                    //OCC中baseIndex是从1开始的，所以这里我们减1
                    --baseIndex;
                    baseEdges[baseIndex].push_back(edgeID);
                    ++edgeID;
                }
                int wireEndEdgeID = edgeID;
                for (int i = wireStartEdgeID; i < wireEndEdgeID; ++i) {
                    if(i == wireStartEdgeID) {
                        bOccEdges[i].prev = wireEndEdgeID - 1;
                        bOccEdges[i].next = i + 1;
                    }
                    else if(i == wireEndEdgeID - 1){
                        bOccEdges[i].prev = i - 1;
                        bOccEdges[i].next = wireStartEdgeID;
                    }
                    else {
						bOccEdges[i].prev = i - 1;
                        bOccEdges[i].next = i + 1;
                    }
                }
            }
            faceEdgeCnt.push_back(edgeID);
            faces.push_back(face);
            ++faceID;
        }
        for (auto& be : baseEdges) {
            if (be.size() == 2) {
                bOccEdges[be[0]].sym = be[1];
                bOccEdges[be[0]].symFaceID = bOccEdges[be[1]].faceID;
                bOccEdges[be[1]].sym = be[0];
                bOccEdges[be[1]].symFaceID = bOccEdges[be[0]].faceID;
            }
        }
    }

	//进行三角化
    BRepMesh_IncrementalMesh mesher(shape, deflection, Standard_True, angle);
    //meshToObjFile("C:/Users/ASUS/Desktop/a.obj");
    
    //helper function。用于从重复的he中选出一条he进行渲染。
    //[&]代表对于没有写在参数里的变量都是引用拿来
    //如果变量写在参数里，像这里的heToEdgeID和symHE，则都是值传递。所以heToEdgeID和symHE在参数中需要改为引用
    auto chooseHe = [&](int n1, int n2, std::unordered_map<std::string, int>& boundaryHEIdx,
        std::unordered_map<std::string, int>& symHE, glm::vec4& normal, std::vector<Vertex>& vertices,
        std::vector<uint32_t>& indices, std::vector<std::vector<glm::vec3>>& keyNors,
        std::unordered_map<std::string, std::pair<int, int>>& keyToKeyNorsIdx) -> bool {
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
            //在keyNors中记录faceNor
            auto it2 = keyToKeyNorsIdx.find(std::to_string(n1) + "#" + std::to_string(n2));
            keyNors[it2->second.first][it2->second.second] = normal;
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
			//不在vertices2中，则记录-1
            //(key, vertices2中的idx), ...
            std::unordered_map<std::string, int> boundaryHEIdx;
            //按照edge顺序记录该face的mesh中的边界边，在该face三角化中的normal
            std::vector<std::vector<glm::vec3>> keyNors;
            //按照edge顺序记录该face的mesh中的边界边，在该face三角化中的uid
            std::vector<std::vector<int>> keyUids;
            //通过该face三角化中的key，找到对应keyNorsTemp
            //(key, <edges Id, 在该edge上的第几个>)
            //(n1+n2, <edges Id, keyNorsTemp Idx>), ....
            std::unordered_map<std::string, std::pair<int, int>> keyToKeyNorsIdx;
            //也是记录bEdge在vertices2中的id，只不过这个严格按照edges的顺序
            //不在vertices2中，则记录-1
            std::vector<std::vector<int>> bEdgesIdx;

            for (int i = faceEdgeCnt[faceID]; i < faceEdgeCnt[faceID + 1]; ++i) {
                bool shouldLoad = true;
                std::vector<int> keyUidsTemp;
                std::vector<glm::vec3> keyNorsTemp;
                std::vector<int> bEdgesIdxTemp;
                //如果该边界edge有sym，渲染id小的那一个
                if (bOccEdges[i].sym != -1) {
                    if (bOccEdges[i].sym < i) {
                        shouldLoad = false;
                        bOccEdges[i].render = false;
                    }
                }
                Handle(Poly_PolygonOnTriangulation) poly =
                    BRep_Tool::PolygonOnTriangulation(bOccEdges[i].edge, tri, loc);
                if (poly.IsNull()) {
                    mesh.bEdgesSE.push_back(-1);
                    mesh.bEdgesSE.push_back(-1);
                    continue;
                }
                int currBEdgeSE1 = mesh.indices2.size();
                const TColStd_Array1OfInteger& polyNodes = poly->Nodes();
                
                //bool needReverse = (face.Orientation() != edges[i].edge.Orientation());
                bool needReverse = (bOccEdges[i].edge.Orientation() == TopAbs_REVERSED);
                if (needReverse) {
                    for (int j = polyNodes.Upper(); j > polyNodes.Lower(); --j) {
                        int n1 = polyNodes(j);
                        int n2 = polyNodes(j - 1);
                        --n1;
                        --n2;

                        std::string key = std::to_string(n1) + "#" + std::to_string(n2);
						//n1#n2: (bLink Id, bLink中第几个bEdge)
                        keyToKeyNorsIdx[key] = std::make_pair<int, int>(i - faceEdgeCnt[faceID], keyNorsTemp.size());
                        //bLink中第几个bEdge: faceNor
                        keyNorsTemp.push_back(glm::vec3(0));
                        //bLink中第几个bEdge: n1,n2
						keyUidsTemp.push_back(n1);
                        keyUidsTemp.push_back(n2);

                        if (shouldLoad) {
                            //n1#n2: 在vertices2中的id
                            boundaryHEIdx[key] = mesh.indices2.size();
							//bLink中第几个bEdge: 在vertices2中的id
                            bEdgesIdxTemp.push_back(mesh.indices2.size());
                            mesh.vertices2.push_back(Vertex(positions[n1], glm::vec4(0), uvs[n1], faceID, i - 1, glm::vec4(0), n1));
                            mesh.vertices2.push_back(Vertex(positions[n2], glm::vec4(0), uvs[n2], faceID, i - 1, glm::vec4(0), n2));
                            mesh.indices2.push_back(mesh.indices2.size());
                            mesh.indices2.push_back(mesh.indices2.size());
                        }
                        else {
                            boundaryHEIdx[key] = -1;
                            bEdgesIdxTemp.push_back(-1);
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
                        keyToKeyNorsIdx[key] = std::make_pair<int, int>(i - faceEdgeCnt[faceID], keyNorsTemp.size());
                        keyNorsTemp.push_back(glm::vec3(0));
                        keyUidsTemp.push_back(n1);
                        keyUidsTemp.push_back(n2);

                        if (shouldLoad) {
                            boundaryHEIdx[key] = mesh.indices2.size();
                            bEdgesIdxTemp.push_back(mesh.indices2.size());
                            mesh.vertices2.push_back(Vertex(positions[n1], glm::vec4(0), uvs[n1], faceID, i - 1, glm::vec4(0), n1));
                            mesh.vertices2.push_back(Vertex(positions[n2], glm::vec4(0), uvs[n2], faceID, i - 1, glm::vec4(0), n2));
                            mesh.indices2.push_back(mesh.indices2.size());
                            mesh.indices2.push_back(mesh.indices2.size());
                        }
                        else {
                            boundaryHEIdx[key] = -1;
                            bEdgesIdxTemp.push_back(-1);
                        }
                    }
                }
                int currBEdgeSE2 = mesh.indices2.size();
                keyNors.push_back(keyNorsTemp);
				keyUids.push_back(keyUidsTemp);
                bEdgesIdx.push_back(bEdgesIdxTemp);
                if (currBEdgeSE1 == currBEdgeSE2) {
                    continue;
                }
                else {
                    mesh.bEdgesSE.push_back(currBEdgeSE1);
                    mesh.bEdgesSE.push_back(currBEdgeSE2);
                    mesh.bEdgeSymObjs.push_back(bOccEdges[i].symFaceID);
                }
            }

            //记录该face的mesh中的sym he
            //[n1#n2] : {vertices2中的id,该面normal}
            bool needReverse = (face.Orientation() == TopAbs_REVERSED);
            std::unordered_map<std::string, int> symHE;
            if (needReverse) {
                for (int i = triangles.Upper(); i >= triangles.Lower(); --i) {
                    int n1, n2, n3; //node indices
                    triangles(i).Get(n1, n2, n3);
                    --n1;
                    --n2;
                    --n3;
                    int temp = n3;
                    n3 = n1;
                    n1 = temp;
                    glm::vec3 v1 = positions[n2] - positions[n1];
                    glm::vec3 v2 = positions[n3] - positions[n1];
                    glm::vec4 normal = glm::vec4(glm::normalize(glm::cross(v1, v2)), 0);
                    mesh.vertices1.push_back(Vertex(positions[n1], normal, uvs[n1], faceID, i - 1, normal, n1));
                    mesh.vertices1.push_back(Vertex(positions[n2], normal, uvs[n2], faceID, i - 1, normal, n2));
                    mesh.vertices1.push_back(Vertex(positions[n3], normal, uvs[n3], faceID, i - 1, normal, n3));
                    mesh.indices1.push_back(mesh.indices1.size());
                    mesh.indices1.push_back(mesh.indices1.size());
                    mesh.indices1.push_back(mesh.indices1.size());
                    if (chooseHe(n1, n2, boundaryHEIdx, symHE, normal, mesh.vertices2, mesh.indices2, keyNors, keyToKeyNorsIdx)) {
                        mesh.vertices2.push_back(Vertex(positions[n1], normal, uvs[n1], faceID, i - 1, normal, n1));
                        mesh.vertices2.push_back(Vertex(positions[n2], normal, uvs[n2], faceID, i - 1, normal, n2));
                        mesh.indices2.push_back(mesh.indices2.size());
                        mesh.indices2.push_back(mesh.indices2.size());
                    }
                    if (chooseHe(n2, n3, boundaryHEIdx, symHE, normal, mesh.vertices2, mesh.indices2, keyNors, keyToKeyNorsIdx)) {
                        mesh.vertices2.push_back(Vertex(positions[n2], normal, uvs[n2], faceID, i - 1, normal, n2));
                        mesh.vertices2.push_back(Vertex(positions[n3], normal, uvs[n3], faceID, i - 1, normal, n3));
                        mesh.indices2.push_back(mesh.indices2.size());
                        mesh.indices2.push_back(mesh.indices2.size());
                    }
                    if (chooseHe(n3, n1, boundaryHEIdx, symHE, normal, mesh.vertices2, mesh.indices2, keyNors, keyToKeyNorsIdx)) {
                        mesh.vertices2.push_back(Vertex(positions[n3], normal, uvs[n3], faceID, i - 1, normal, n3));
                        mesh.vertices2.push_back(Vertex(positions[n1], normal, uvs[n1], faceID, i - 1, normal, n1));
                        mesh.indices2.push_back(mesh.indices2.size());
                        mesh.indices2.push_back(mesh.indices2.size());
                    }

                }
            }
            else {
                for (int i = triangles.Lower(); i <= triangles.Upper(); ++i) {
                    int n1, n2, n3; //node indices
                    triangles(i).Get(n1, n2, n3);
                    --n1;
                    --n2;
                    --n3;
                    glm::vec3 v1 = positions[n2] - positions[n1];
                    glm::vec3 v2 = positions[n3] - positions[n1];
                    glm::vec4 normal = glm::vec4(glm::normalize(glm::cross(v1, v2)), 0);
                    mesh.vertices1.push_back(Vertex(positions[n1], normal, uvs[n1], faceID, i - 1, normal, n1));
                    mesh.vertices1.push_back(Vertex(positions[n2], normal, uvs[n2], faceID, i - 1, normal, n2));
                    mesh.vertices1.push_back(Vertex(positions[n3], normal, uvs[n3], faceID, i - 1, normal, n3));
                    mesh.indices1.push_back(mesh.indices1.size());
                    mesh.indices1.push_back(mesh.indices1.size());
                    mesh.indices1.push_back(mesh.indices1.size());
                    if (chooseHe(n1, n2, boundaryHEIdx, symHE, normal, mesh.vertices2, mesh.indices2, keyNors, keyToKeyNorsIdx)) {
                        mesh.vertices2.push_back(Vertex(positions[n1], normal, uvs[n1], faceID, i - 1, normal, n1));
                        mesh.vertices2.push_back(Vertex(positions[n2], normal, uvs[n2], faceID, i - 1, normal, n2));
                        mesh.indices2.push_back(mesh.indices2.size());
                        mesh.indices2.push_back(mesh.indices2.size());
                    }
                    if (chooseHe(n2, n3, boundaryHEIdx, symHE, normal, mesh.vertices2, mesh.indices2, keyNors, keyToKeyNorsIdx)) {
                        mesh.vertices2.push_back(Vertex(positions[n2], normal, uvs[n2], faceID, i - 1, normal, n2));
                        mesh.vertices2.push_back(Vertex(positions[n3], normal, uvs[n3], faceID, i - 1, normal, n3));
                        mesh.indices2.push_back(mesh.indices2.size());
                        mesh.indices2.push_back(mesh.indices2.size());
                    }
                    if (chooseHe(n3, n1, boundaryHEIdx, symHE, normal, mesh.vertices2, mesh.indices2, keyNors, keyToKeyNorsIdx)) {
                        mesh.vertices2.push_back(Vertex(positions[n3], normal, uvs[n3], faceID, i - 1, normal, n3));
                        mesh.vertices2.push_back(Vertex(positions[n1], normal, uvs[n1], faceID, i - 1, normal, n1));
                        mesh.indices2.push_back(mesh.indices2.size());
                        mesh.indices2.push_back(mesh.indices2.size());
                    }

                }
            }
            
            bEdgesNor.insert(bEdgesNor.end(), keyNors.begin(), keyNors.end());
			bEdgeUids.insert(bEdgeUids.end(), keyUids.begin(), keyUids.end());
            this->bEdgesIdx.insert(this->bEdgesIdx.end(), bEdgesIdx.begin(), bEdgesIdx.end());
        }

		//给bEdge赋予symFaceNor
        for (int edgeId = 0; edgeId < bEdgesIdx.size(); ++edgeId) {
            int sym = bOccEdges[edgeId].sym;
            if (sym != -1) {
                for (int i = 0; i < bEdgesIdx[edgeId].size(); ++i) {
                    //这个存储着vertices2里的id，如果不为-1，说明是要渲染的bEdge
                    if (bEdgesIdx[edgeId][i] != -1) {
                        glm::vec3 symNor = bEdgesNor[sym][bEdgesNor[sym].size() - 1 - i];
                        int symn1 = bEdgeUids[sym][bEdgeUids[sym].size() - 1 - i * 2];
                        int symn2 = bEdgeUids[sym][bEdgeUids[sym].size() - 1 - i * 2 - 1];
                        mesh.vertices2[bEdgesIdx[edgeId][i]].symFaceNor = symNor;
                        mesh.vertices2[bEdgesIdx[edgeId][i] + 1].symFaceNor = symNor;
                        mesh.vertices2[bEdgesIdx[edgeId][i]].border = 1;
                        mesh.vertices2[bEdgesIdx[edgeId][i] + 1].border = 1;
                        mesh.vertices2[bEdgesIdx[edgeId][i]].symUid = symn1;
                        mesh.vertices2[bEdgesIdx[edgeId][i] + 1].symUid = symn2;
                    }
                }
            }
            else {
                for (int i = 0; i < bEdgesIdx[edgeId].size(); ++i) {
                    if (bEdgesIdx[edgeId][i] != -1) {
                        mesh.vertices2[bEdgesIdx[edgeId][i]].border = 2;
                        mesh.vertices2[bEdgesIdx[edgeId][i] + 1].border = 2;
                    }
                }
            }
        }


        for (int edgeId = 0; edgeId < bEdgesIdx.size(); ++edgeId) {
            if (bOccEdges[edgeId].render) {
                std::vector<int> startConnected, endConnected;
                std::vector<int> startConnectedAway, endConnectedAway;
                findAllConnectedEdges(edgeId, bEdgesIdx, bOccEdges, startConnected, startConnectedAway, endConnected, endConnectedAway);
                mesh.bEdgeConnects.insert(mesh.bEdgeConnects.end(), startConnected.begin(), startConnected.end());
                mesh.bEdgeConnects.insert(mesh.bEdgeConnects.end(), endConnected.begin(), endConnected.end());
                mesh.bEdgeConnectsAway.insert(mesh.bEdgeConnectsAway.end(), startConnectedAway.begin(), startConnectedAway.end());
				mesh.bEdgeConnectsAway.insert(mesh.bEdgeConnectsAway.end(), endConnectedAway.begin(), endConnectedAway.end());
            }
        }

    }
};