/*
 * Student solution for UC Berkeley Project 2
 *
 * Implemented by Tim Greeno on 2/2016.
 *
 */

#include "student_code.h"
#include "mutablePriorityQueue.h"

namespace CGL {

    void BezierPatch::preprocess() {
        // TODO Part 1.
        // TODO If you use the matrix form for Bezier patch evaluation, you will need to
        // TODO compute your matrices based on the 16 control points here. 
        // TODO You will also need to define your matrices
        // TODO as member variables in the "BezierPatch" class.
        // TODO If you use De Casteljau's recursive algorithm, you will not need to do anything here.

        double data[16] = {1.0,0.0,0.0,0.0,-3.0,3.0,0.0,0.0,3.0,-6.0,3.0,0.0,-1.0,3.0,-3.0,1.0};
        double xdata[16];
        double ydata[16];
        double zdata[16];

        Matrix4x4 left = Matrix4x4(data);
        Matrix4x4 right = left.T();

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                xdata[i*4 + j] = controlPoints[i][j][0];
                ydata[i*4 + j] = controlPoints[i][j][1];
                zdata[i*4 + j] = controlPoints[i][j][2];
            }
        }
        X = Matrix4x4(xdata);
        Y = Matrix4x4(ydata);
        Z = Matrix4x4(zdata);

        X = left * X;
        X = X * right;
        Y = left * Y;
        Y = Y * right;
        Z = left * Z;
        Z = Z * right;
    }

    Vector3D BezierPatch::evaluate(double u, double v) const {
        // TODO Part 1.
        // TODO Returns the 3D point whose parametric coordinates are (u, v) on the Bezier patch.
        // TODO Note that both u and v are within [0, 1]. 
        
        Vector4D U = Vector4D(1.0, u, pow(u, 2.0), pow(u, 3.0));
        Vector4D V = Vector4D(1.0, v, pow(v, 2.0), pow(v, 3.0));

        double xval = dot(U,X*V);
        double yval = dot(U,Y*V);
        double zval = dot(U,Z*V);
        return Vector3D(xval,yval,zval);
    }

    void BezierPatch::add2mesh(Polymesh* mesh) const {
        // TODO Part 1.
        // TODO Tessellate the given Bezier patch into triangles uniformly on a 8x8 grid(8x8x2=128 triangles) in parameter space.
        // TODO You will call your own evaluate function here to compute vertex positions of the tessellated triangles.
        // TODO The "addTriangle" function inherited from the "BezierPatchLoader" class may help you add triangles to the output mesh. 
        for (double u = 0.0; u < 1.00; u += .125) {
            for (double v = 0.0; v < 1.00; v += .125) {
                Vector3D v0 = evaluate(u, v);
                Vector3D v1 = evaluate(u+.125,v+.125);
                Vector3D v2 = evaluate(u,v+.125);
                addTriangle(mesh,v0,v1,v2);
                v2 = evaluate(u+.125,v);
                addTriangle(mesh,v0,v2,v1);
            }
        }
    }

    Vector3D Vertex::normal(void) const
    // TODO Part 2.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    {
        // TODO Compute and return the area-weighted unit normal.
        Vector3D n(0,0,0); 
        HalfedgeCIter h = halfedge(); 
        h = h->twin(); 

        HalfedgeCIter horig = h;
        do {
            Vector3D v0 = h->vertex()->position;
            h = h->next();
            Vector3D v1 = h->vertex()->position;
            h = h->twin();
            Vector3D v2 = h->vertex()->position;

            n = n + cross(v1-v0,v2-v1);

        } while (h != horig);
        return n.unit();
    }

    void Vertex::computeCentroid(void) {
        Vector3D avg = Vector3D();
        HalfedgeCIter h = halfedge(); 
        h = h->twin(); 
        HalfedgeCIter horig = h;
        do {
            Vector3D v0 = h->vertex()->position;
            h = h->next();
            h = h->twin();
            avg += v0;

        } while (h != horig);
        centroid = avg / (float) degree();
    }

    std::vector<EdgeIter> deletedEdges;


    VertexIter HalfedgeMesh::collapseEdge( EdgeIter e0) 
    {
        // TODO This method should collapse the given edge and return an iterator to the new vertex created by the collapse
        

        if (e0->isBoundary()) {
            return VertexIter();
        }
        HalfedgeIter h0 = e0->halfedge();
        HalfedgeIter h1 = h0->twin();
        HalfedgeIter h2 = h0->next();
        HalfedgeIter h3 = h1->next();
        HalfedgeIter h5 = h2->next(); 
        HalfedgeIter h4 = h5->twin();
        HalfedgeIter h6 = h3->twin();
        HalfedgeIter h7 = h3->next();
        HalfedgeIter h8 = h4->next();
        HalfedgeIter h9 = h2->twin();
        HalfedgeIter h10 = h7->twin();
        VertexIter v0 = h0->vertex();
        VertexIter v1 = h1->vertex();
        VertexIter v2 = h9->vertex();
        VertexIter v3 = h6->vertex();
        FaceIter f0 = h0->face();
        FaceIter f1 = h1->face();
        EdgeIter e2 = h5->edge();
        EdgeIter e3 = h3->edge();
        EdgeIter e4 = h9->edge();
        EdgeIter e5 = h10->edge();

        /////////////////Number of neighbors check START/////////////////

        HalfedgeCIter ch = e0->halfedge();
        HalfedgeCIter dh = e0->halfedge()->twin();
        int neighborcount = 0;
        int loopcount = 0;
         do {
        //cout << "forever 7" << endl;
             HalfedgeCIter chtwin = ch->twin();
             VertexCIter cneighbor = chtwin->vertex();
             int count = 0;
             do {
        //cout << "forever 8" << endl;
                 HalfedgeCIter dhtwin = dh->twin();
                 VertexCIter vneighbor = dhtwin->vertex();
                 if (cneighbor == vneighbor) {
                     neighborcount += 1;
                 }
                 dh = dhtwin->next();
             } while (dh != e0->halfedge()->twin());
             ch = chtwin->next();
         } while (ch != e0->halfedge());
 
         //cout << "Number of shared neighbors = " << neighborcount << endl;
         if (neighborcount != 2) {
            cout << "WARNING: collapse aborted due to more than one shared neighbor vertex.";
             return VertexIter();
         }

        /////////////////Number of neighbors check END/////////////////

        /////////////////Triangle flip check START/////////////////

        Vector3D v1newpos = (v0->position + v1->position) / 2.0;

        HalfedgeCIter leftmove = h9;
        HalfedgeCIter leftstop = h7;
        do {
        //cout << "forever 9" << endl;
            Vector3D vertexApos = leftmove->vertex()->position;
            Vector3D vertexBpos = leftmove->next()->next()->vertex()->position;
            Vector3D beforenormal = cross(vertexApos - v1->position, vertexBpos - v1->position);
            Vector3D afternormal = cross(vertexApos - v1newpos, vertexBpos - v1newpos);
            if (dot(beforenormal, afternormal) < 0) {
                cout << "WARNING: collapse aborted due to a flipped triangle.";
                return VertexIter();
            }
            leftmove = leftmove->next()->twin();
        } while (leftmove != leftstop);


        HalfedgeCIter rightmove = h6;
        HalfedgeCIter rightstop = h5;
        do {
            Vector3D vertexApos = rightmove->vertex()->position;
            Vector3D vertexBpos = rightmove->next()->next()->vertex()->position;
            Vector3D beforenormal = cross(vertexApos - v0->position, vertexBpos - v0->position);
            Vector3D afternormal = cross(vertexApos - v1newpos, vertexBpos - v1newpos);

            if (dot(beforenormal, afternormal) < 0) {
                cout << "WARNING: collapse aborted due to a flipped triangle.";
                return VertexIter();
            }
            rightmove = rightmove->next()->twin();
        } while (rightmove != rightstop);
        /////////////////Triangle flip check END/////////////////


        HalfedgeIter hstop = h5;
        HalfedgeIter hmove = h6;
        do {
            hmove = hmove->next();
            hmove->vertex() = v1;
            hmove = hmove->twin();
            //cout << &*hmove << endl;
            //cout << &*hstop << endl;
        } while (hmove != hstop);

        //Assign new position and halfedge to the shifted v1
        v1->position = (v0->position + v1->position) / 2.0;
        v1->halfedge() = h10;
        //Assign twins across deleted triangles
        h4->twin() = h9;
        h9->twin() = h4;
        h10->twin() = h6;
        h6->twin() = h10;
        //Assign halfedges to the remaining edges
        e4->halfedge() = h4;
        e5->halfedge() = h6;
        //Assign new halfedges for top & bot vertices
        v2->halfedge() = h9;
        v3->halfedge() = h6;
        //For the halfedges whose edges are to be removed, assign them new edges
        h4->edge() = e4;
        h6->edge() = e5;
        //Delete everything within the collapsed space

        deletedEdges.push_back(e0);
        deletedEdges.push_back(e3);
        deletedEdges.push_back(e2);

        deleteVertex(v0);//
        deleteFace(f0);//
        deleteFace(f1);//
        deleteEdge(e0);//
        deleteEdge(e3);//
        deleteEdge(e2);//
        deleteHalfedge(h0);
        deleteHalfedge(h1);
        deleteHalfedge(h2);
        deleteHalfedge(h5);
        deleteHalfedge(h7);
        deleteHalfedge(h3);
        // cout << "finishing collapse " << endl;
        return v1;
    }



    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
        // TODO Part 3.
        // TODO This method should flip the given edge and return an iterator to the flipped edge.
        HalfedgeIter h1 = e0->halfedge();
        HalfedgeIter t1 = h1->twin();

        if (e0->isBoundary()) {
            return e0;
        }

        HalfedgeIter h2 = h1->next();
        HalfedgeIter t2 = t1->next();        
        HalfedgeIter h3 = h2->next();
        HalfedgeIter t3 = t2->next();        

        VertexIter v0 = h1->vertex();
        VertexIter v2 = t1->vertex();
        VertexIter v1 = h3->vertex();
        VertexIter v3 = t3->vertex();

        FaceIter f1 = h1->face();
        FaceIter f2 = t1->face();

        h1->setNeighbors(t3,t1,v1,e0,f1);
        h2->setNeighbors(h1,h2->twin(),v2,h2->edge(),f1);
        h3->setNeighbors(t2,h3->twin(),v1,h3->edge(),f2);        
        t1->setNeighbors(h3,h1,v3,e0,f2);
        t2->setNeighbors(t1,t2->twin(),v0,t2->edge(),f2);
        t3->setNeighbors(h2,t3->twin(),v3,t3->edge(),f1);

        f1->halfedge() = h1;
        f2->halfedge() = t1;
        v0->halfedge() = t2;
        v1->halfedge() = h3;
        v2->halfedge() = h2;
        v3->halfedge() = t1;
        return e0;
    }

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
        // TODO Part 4.
        // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
        // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
        if (e0->isBoundary()) {
            return VertexIter();
        }
        HalfedgeIter h1 = e0->halfedge();
        HalfedgeIter t1 = h1->twin();
        HalfedgeIter h2 = h1->next();
        HalfedgeIter t2 = t1->next();        
        HalfedgeIter h3 = h2->next();
        HalfedgeIter t3 = t2->next();        
        VertexIter v0 = h1->vertex();
        VertexIter v2 = t1->vertex();
        VertexIter v1 = h3->vertex();
        VertexIter v3 = t3->vertex();
        FaceIter f1 = h1->face();
        FaceIter f2 = t1->face();

        VertexIter v4 = newVertex();
        Vector3D pos = ((v0->position) + (v2->position)) * .50;
        v4->position = pos;

        HalfedgeIter h4 = newHalfedge();
        HalfedgeIter h5 = newHalfedge();
        HalfedgeIter h6 = newHalfedge();
        HalfedgeIter t4 = newHalfedge();
        HalfedgeIter t5 = newHalfedge();
        HalfedgeIter t6 = newHalfedge();
        EdgeIter e1 = newEdge();
        EdgeIter e2 = newEdge();
        EdgeIter e3 = newEdge();
        FaceIter f3 = newFace();
        FaceIter f4 = newFace();

        h1->setNeighbors(h4,t6,v0,e0,f1);
        h2->setNeighbors(h5,h2->twin(),v2,h2->edge(),f3);
        h3->setNeighbors(h1,h3->twin(),v1,h3->edge(),f1);        
        h4->setNeighbors(h3,h5,v4,e1,f1);        
        h5->setNeighbors(h6,h4,v1,e1,f3);        
        h6->setNeighbors(h2,t1,v4,e2,f3);

        t1->setNeighbors(t4,h6,v2,e2,f2);
        t2->setNeighbors(t5,t2->twin(),v0,t2->edge(),f4);
        t3->setNeighbors(t1,t3->twin(),v3,t3->edge(),f2);        
        t4->setNeighbors(t3,t5,v4,e3,f2);        
        t5->setNeighbors(t6,t4,v3,e3,f4);        
        t6->setNeighbors(t2,h1,v4,e0,f4);  

        f1->halfedge() = h1;
        f2->halfedge() = t1;
        f3->halfedge() = h5;
        f4->halfedge() = t5;

        v0->halfedge() = t2;
        v1->halfedge() = h3;
        v2->halfedge() = h2;
        v3->halfedge() = t3; 
        v4->halfedge() = h6; 

        e0->halfedge() = h1;       
        e1->halfedge() = h5;       
        e2->halfedge() = t1;       
        e3->halfedge() = t5;       

        return v4;
    }


    void MeshResampler::upsample(HalfedgeMesh& mesh)
    // TODO Part 5.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    {
        // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
        // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
        // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
        // the new subdivided (fine) mesh, which has more elements to traverse.  We will then assign vertex positions in
        // the new mesh based on the values we computed for the original mesh.


        // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
        // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
        // TODO a vertex of the original mesh.
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
            v->isNew = false;
            float n = (float) v->degree();
            float u = (n == 3.0) ? 3.0 / 16.0 : 3.0 / (8.0 * (float) n);
            
            Vector3D sum = Vector3D(0,0,0);
            HalfedgeIter h1 = v->halfedge();
            HalfedgeIter h = v->halfedge();
            VertexIter curr;
            do {
                HalfedgeIter h_twin = h->twin(); 
                VertexIter curr = h_twin->vertex();
                sum = sum + curr->position;  
                h = h_twin->next();         
            } while(h != h1); 

            v->newPosition = (1.0 - n * u) * v->position + (u * sum);
        }

        // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            HalfedgeIter h1 = e->halfedge();
            HalfedgeIter t1 = h1->twin();

            HalfedgeIter h3 = h1->next()->next();
            HalfedgeIter t3 = t1->next()->next();
        
            VertexIter v0 = h1->vertex();
            VertexIter v2 = t1->vertex();
            VertexIter v1 = h3->vertex();
            VertexIter v3 = t3->vertex();

            e->newPosition = (3.0/8.0) * (v0->position + v2->position) + (1.0 / 8.0) * (v1->position + v3->position);

            e->isNew = false;
        }

        // TODO Next, we're going to split every edge in the mesh, in any order.  For future
        // TODO reference, we're also going to store some information about which subdivided
        // TODO edges come from splitting an edge in the original mesh, and which edges are new,
        // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
        // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
        // TODO just split (and the loop will never end!)


        EdgeIter last_edge;
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            last_edge = e;
        }
        for (EdgeIter e = mesh.edgesBegin(); e != last_edge; e++) {
                VertexIter newv = mesh.splitEdge(e);
                newv->isNew = true;
                newv->newPosition = e->newPosition;

                HalfedgeIter h6 = newv->halfedge();

                h6->edge()->isNew = false;

                h6->twin()->next()->twin()->next()->edge()->isNew = false;

                h6->twin()->next()->edge()->isNew = true;

                h6->next()->next()->edge()->isNew = true;

        }
         if(last_edge->isNew == false) {
                VertexIter newv = mesh.splitEdge(last_edge);
                newv->isNew = true;
                newv->newPosition = last_edge->newPosition;

                HalfedgeIter h1 = newv->halfedge();

                h1->edge()->isNew = false;

                h1->twin()->next()->twin()->next()->edge()->isNew = false;

                h1->twin()->next()->edge()->isNew = true;

                h1->next()->next()->edge()->isNew = true;

             }



        // TODO Now flip any new edge that connects an old and new vertex.
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            if (e->isNew == true) {
                HalfedgeIter h1 = e->halfedge();
                HalfedgeIter t1 = h1->twin();
            
                VertexIter v0 = h1->vertex();
                VertexIter v2 = t1->vertex();
                if((v0->isNew == false && v2->isNew == true) || (v0->isNew == true && v2->isNew == false)) {
                    mesh.flipEdge(e);
                }
            }
        }

        // TODO Finally, copy the new vertex positions into final Vertex::position.
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
            v->position = v->newPosition;
        }
    }
    void MeshResampler::simplify(HalfedgeMesh& mesh) {
        MutablePriorityQueue<EdgeRecord> queue;
        //Step 1: Compute a quadric for each face
        int tcount = 0;
        
        for (FaceIter f = mesh.facesBegin(); f != mesh.facesEnd(); f++) {
            Vector3D N = f->normal();
            double d = dot(-N, f->halfedge()->vertex()->position);
            Vector4D v = Vector4D(N.x, N.y, N.z, d);
            f->quadric = outer(v, v);
            tcount += 1;
        }
        printf("%d\n", tcount);
        //Step 2: Compute a quadric for each vertex
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
            v->computeQuadric();
        }
        //Step 3: For each edge, create an EdgeRecord and insert it into one global MutablePriorityQueue
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            EdgeRecord myRecord(e); //ASK ABOUT HOW TO ADD/REMOVE FROM QUEUE
            queue.insert(e);
        }

        while (mesh.nFaces() >= tcount * 0.25) {
            printf("%lu\n", mesh.nFaces());
            //Find and remove the cheapest edge from the queue
            EdgeRecord bestEdge = queue.top();
            queue.pop();
            //Compute the new Quadric by summing the quadric at its two endpoints
            VertexIter v0 = bestEdge.edge->halfedge()->vertex();
            VertexIter v1 = bestEdge.edge->halfedge()->twin()->vertex();
            Matrix4x4 newQuadric = v0->quadric + v1->quadric;

            //Remove any edge touching either of its endpoints from the queue
            
            //v0
            HalfedgeIter h0 = v0->halfedge(); 
            h0 = h0->twin(); 

            HalfedgeIter horig0 = h0;
            do {
                EdgeIter e0 = h0->edge();
                EdgeRecord EdgeRecord(e0);
                queue.remove(e0);
                h0 = h0->next();
                h0 = h0->twin();
            } while (h0 != horig0);

            //v1
            HalfedgeIter h1 = v1->halfedge(); 
            h1 = h1->twin(); 

            HalfedgeIter horig1 = h1;
            do {
                EdgeRecord myRecord(h1->edge());
                queue.remove(h1->edge());
                h1 = h1->next();
                h1 = h1->twin();
            } while (h1 != horig1);

            //Collapse the edge and set the quadric of the new vertex to the new quadric computed above.
            VertexIter newVertex = mesh.collapseEdge(bestEdge.edge);
            //VertexIter newVertex = VertexIter();
            newVertex->quadric = newQuadric;

            //Insert any edge touching the new vertex into the queue, creating new edge records for each of them
            HalfedgeIter h2 = newVertex->halfedge(); 
            h2 = h2->twin(); 
            printf("%s\n", "here1");
            HalfedgeIter horig2 = h2;
            do {
                EdgeRecord myRecord(h2->edge());
                queue.insert(h2->edge());
                h2 = h2->next();
                h2 = h2->twin();
            } while (h2 != horig2);
            printf("%s\n", "here2");
        }       
    }

    void MeshResampler::remesh(HalfedgeMesh& mesh) {

        cout << "Remeshing..." << "\n";

        double l = 0.0;
        int count = 0;
        // Calculate average edge length
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            l += e->length();
            count++;
        }
        l = l / (double) count;
        // Split edges longer than (4/3)l
        std::vector<EdgeIter> toCollapse;
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            if (e->length() > 4.0/3.0 * l) {
                mesh.splitEdge(e);
            }
        }

        EdgeIter e = mesh.edgesBegin(); 
        while (e != mesh.edgesEnd()) {
            // EdgeIter e = e1;
            if (e->length() < 4.0/5.0 * l) {
                // cout << "about to collapse " << "\n";
                // mesh.collapseEdge(e);
                // cout << "re-entering remesh" << endl;

                // toCollapse.push_back(e); //Tim's code

                // mesh.collapseEdge(e); //Kenny's code
                // e = mesh.edgesBegin(); 

                // if (std::find(deletedEdges.begin(), deletedEdges.end(), e) != deletedEdges.end()) {
                //     // Found e in deleted edges, therefore must advance until safe
                // }



                while (std::find(deletedEdges.begin(), deletedEdges.end(), e) != deletedEdges.end()) {
                    cout << "found a deleted edge" << endl;
                    e++;
                }
                mesh.collapseEdge(e);
                     

            }
            // advance(e, rand() % mesh.nEdges() + 0); //advance the iterator by 1+ or more
            e++;

            // cout << "difference between e1 and begin " << std::distance(mesh.edgesBegin(), e1) << "\n";
            // cout << "difference between end and begin " << std::distance(mesh.edgesBegin(), mesh.edgesEnd()) << "\n";
        }
        // cout << "forever 4 " << endl;
        // for (EdgeIter e : toCollapse) {
        //     if (mesh.containsEdge(e)) {
        //         cout << "about to collapse " << "\n";
        //         mesh.collapseEdge(e);
        //     }
        // }

    
        // Flip edges for variance improvement
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            int a1 = e->halfedge()->vertex()->degree();
            int a2 = e->halfedge()->twin()->vertex()->degree();
            int b1 = e->halfedge()->next()->next()->vertex()->degree();
            int b2 = e->halfedge()->twin()->next()->next()->vertex()->degree();
            int oldvar = abs(a1 - 6) + abs(a2 - 6) + abs(b1 - 6) + abs(b2 - 6);
            int newvar = abs(a1-1-6) + abs(a2-1-6) + abs(b1+1-6) + abs(b2+1-6);
            if (newvar < oldvar) {
                mesh.flipEdge(e);
            }
        }
        for (int i = 0; i < 10; i++) {
            // Compute centroids
            for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
                v->computeCentroid();
            }
            // Move vertices
            for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
                double weight = 1.0 / 5.0;
                Vector3D dir = (v->centroid - v->position);
                v->position = v->position + weight * (dir - dot(v->normal(), dir) * v->normal());
            }
        }
        cout << "Finished remeshing." << endl;
    }
}
