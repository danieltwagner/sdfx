//-----------------------------------------------------------------------------
/*

Simplify a triangle mesh without reducing fidelity by removing interior
triangles for a given group of neighboring coplanar triangles.

*/
//-----------------------------------------------------------------------------

package sdf

import (
	"fmt"
	"math"
)

//-----------------------------------------------------------------------------

type EdgeV3 struct {
	Start, End V3
}
type EdgeV3i struct {
	Start, End V3i
}

func simplifyMesh(m []*Triangle3, meshInc, tolerance float64) ([]*Triangle3, error) {
	fmt.Printf("Simplifying %d triangles...\n", len(m))

	faces := make([][]*Triangle3, len(m)) // faces made up of triangles
	v3iToTriangles := make(map[V3i][]int) // integer vertex to triangle indices
	triToFace := make(map[int]int)        // triangle index to face index
	faceToTris := make(map[int][]int)     // face index to triangle index

	for i, t := range m {
		for _, v := range t.V {
			v3iToTriangles[v3ToV3i(v, meshInc)] = append(v3iToTriangles[v3ToV3i(v, meshInc)], i)
		}
		faces[i] = append(faces[i], t)
		triToFace[i] = i
		faceToTris[i] = []int{i}
	}

	for i, t := range m {
		// Find all neighbors for each triangle (triangles that share at least one vertex) ...
		allNeighbors := make(map[int]bool)
		for _, v := range t.V {
			for _, n := range v3iToTriangles[v3ToV3i(v, meshInc)] {
				allNeighbors[n] = true
			}
		}

		// ... and merge into a face if they share an edge and normal.
		thisFace := triToFace[i]
		for n, _ := range allNeighbors {
			if n == i {
				continue // each triangle is incident to its own vertices
			}
			if shareEdgeAndNormal(t, m[n], tolerance) {
				otherFace := triToFace[n]
				if otherFace == thisFace {
					continue
				}
				// move all triangles from otherFace to this face
				faces[thisFace] = append(faces[thisFace], faces[otherFace]...)
				faces[otherFace] = nil

				// update index-based data structures
				for _, o := range faceToTris[otherFace] {
					triToFace[o] = thisFace
				}
				faceToTris[thisFace] = append(faceToTris[thisFace], faceToTris[otherFace]...)
				faceToTris[otherFace] = nil
			}
		}
	}

	// Now that we have identified the faces, remove all edges that are not needed to describe
	// the shape. That is, remove edges where all incident triangles belong to one face (those
	// are inner edges within a single plane). Also remove vertices where all incident triangles
	// belong to exactly two faces (those are vertices along a straight edge between the two faces,
	// meaning they are coplanar in both faces and therefore colinear along the edge).
	innerEdges := make(map[EdgeV3i]bool)
	extraEdgeVertices := make(map[V3i]bool)
	for i, t := range m {
		for j, v := range t.V {
			v3i := v3ToV3i(v, meshInc)

			// Check if this is an extra edge vertice
			allIncidentTris := v3iToTriangles[v3i]
			facesOfIncidentTris := make(map[int]bool)
			for _, t := range allIncidentTris {
				facesOfIncidentTris[triToFace[t]] = true
				if len(facesOfIncidentTris) > 2 {
					break
				}
			}

			// If we found exactly two incident faces this vertex is part of a longer edge
			// and can be removed later.
			if len(facesOfIncidentTris) == 2 {
				extraEdgeVertices[v3i] = true
			}

			// Check if this is an inner edge
			nextV := t.V[(j + 1) % 3]
			nextV3i := v3ToV3i(nextV, meshInc)
			allIncidentTrisNextV := v3iToTriangles[nextV3i]
			if isInnerEdge(i, allIncidentTris, allIncidentTrisNextV, triToFace) {
				innerEdges[EdgeV3i{v3i, nextV3i}] = true
			}
		}
	}
	fmt.Printf("Will remove %d inner edges and %d edge vertices...\n", len(innerEdges), len(extraEdgeVertices))

	cnt := 0
	var result []*Triangle3
	for _, tris := range faces {
		if len(tris) == 1 {
			result = append(result, tris...)
			cnt++
		} else if len(tris) > 1 {
			reduced, err := removeVertices(tris, innerEdges, extraEdgeVertices, meshInc)
			if err != nil {
				return nil, err
			}

			result = append(result, reduced...)
			cnt++
		}
	}
	fmt.Printf("Simplified mesh has %d triangles (%d faces)\n", len(result), cnt)

	return result, nil
}

func isInnerEdge(tri int, incidentTrisFirstV, incidentTrisNextV []int, triToFace map[int]int) bool {
	for _, i := range incidentTrisFirstV {
		if i == tri {
			continue
		}
		for _, j := range incidentTrisNextV {
			if i == j {
				// We found another triangle that shares this edge
				if triToFace[i] != triToFace[tri] {
					return false
				}
			}
		}
	}
	return true
}

func shareEdgeAndNormal(t1 *Triangle3, t2 *Triangle3, tolerance float64) bool {
	// check normals
	if !t1.Normal().Equals(t2.Normal(), tolerance) {
		return false
	}

	sharedVertices := 0
	for _, v1 := range t1.V {
		for _, v2 := range t2.V {
			if v1.Equals(v2, tolerance) {
				sharedVertices++
				break
			}
		}
	}
	return sharedVertices > 1
}

func v3ToV3i(v V3, meshInc float64) V3i {
	// given a mesh increment, turn a floating point vertex into an integer
	// vertex such that there are hopefully few collisions
	return V3i{int(v.X / meshInc * 100.0), int(v.Y / meshInc * 100.0), int(v.Z / meshInc * 100.0)}
}

func removeVertices(
	m []*Triangle3,
	innerEdges map[EdgeV3i] bool,
	extraEdgeVertices map[V3i]bool,
	meshInc float64,
) ([]*Triangle3, error) {

	// Step 1: Remove inner vertices only
	// Create an index of vertices to edges that we want to keep
	vToEs := make(map[V3i][]EdgeV3)
	for _, t := range m {
		for i, v := range t.V {
			v3i := v3ToV3i(v, meshInc)
			nextV := t.V[(i + 1) % 3]
			nextV3i := v3ToV3i(nextV, meshInc)
			if !innerEdges[EdgeV3i{v3i, nextV3i}] {
				vToEs[v3i] = append(vToEs[v3i], EdgeV3{v, nextV})
			}
		}
	}

	// Step 2: Find loops in vToEs, removing taken edges. We will also collapse the previously
	// marked redundant colinear vertices along the edges to produce a minimal polygon.

	// Project remaining vertices onto the face's plane to construct a 2D polygon.
	origin := m[0].V[0]
	normal := m[0].Normal()

	xDirection := m[0].V[1]
	if origin.Equals(xDirection, epsilon) {
		return nil, fmt.Errorf("first triangle is degenerate, can't form plane: %s", m[0])
	}

	// // Pick a reference point that isn't origin so we can form two axes on our plane
	// var xDirection V3
	// for _, edges := range vToEs {
	// 	if edges[0].Start.Sub(origin).Length() > epsilon {
	// 		xDirection = edges[0].Start
	// 		break
	// 	}
	// }

	planeXaxis := xDirection.Sub(origin).Normalize()
	planeYaxis := normal.Cross(planeXaxis).Normalize()

	var start V3
	var polys []*Polygon
	for len(vToEs) > 0 {
		for _, edges := range vToEs {
			start = edges[0].Start
			break
		}

		p := NewPolygon()
		v3isInPolygon := make(map[V3i]bool)
		for {
			v3i := v3ToV3i(start, meshInc)
			if !extraEdgeVertices[v3i] && !v3isInPolygon[v3i] {
				// project start onto the face's plane
				x := start.Sub(origin).Dot(planeXaxis)
				y := start.Sub(origin).Dot(planeYaxis)
				p.Add(x, y)
				v3isInPolygon[v3i] = true
			}

			outboundEdges := vToEs[v3i]
			if len(outboundEdges) == 0 {
				break // we have completed our polygon
			} else if len(outboundEdges) > 1 {
				err := fmt.Errorf(
					"Found %d outbound edges for vertex %s. Refusing to simplify.\n",
					len(outboundEdges), start,
				)
				return nil, err
			}

			start = outboundEdges[0].End
			delete(vToEs, v3i)
		}
		polys = append(polys, p)
	}

	var toTriangulate *Polygon
	var err error

	if len(polys) == 1 {
		// No holes in this face
		toTriangulate = polys[0]
	} else {
		// Our face has holes. Find the main polygon
		var mainPoly *Polygon
		minX := math.MaxFloat64
		minY := math.MaxFloat64
		maxX := -math.MaxFloat64
		maxY := -math.MaxFloat64
		for _, p := range polys {
			for _, v := range p.Vertices() {
				if v.X < minX  {
					minX = v.X
					mainPoly = p
				}
				if v.X > maxX {
					maxX = v.X
					mainPoly = p
				}
				if v.Y < minY {
					minY = v.Y
					mainPoly = p
				}
				if v.Y > maxY {
					maxY = v.Y
					mainPoly = p
				}
			}
		}

		var holes []*Polygon
		for _, p := range polys {
			if p != mainPoly {
				holes = append(holes, p)
			}
		}
		toTriangulate, err = mergeHoles(mainPoly, holes)
		if err != nil {
			return nil, err
		}
		fmt.Println("POLYGON WITH HOLES COMING")
	}

	t, err := triangulate(toTriangulate)
	if err != nil {
		return nil, err
	}

	// translate back into 3D world coordinates
	var result []*Triangle3
	for _, tri := range(t) {
		v0 := origin.Add(planeXaxis.MulScalar(tri[0].X).Add(planeYaxis.MulScalar(tri[0].Y)))
		v1 := origin.Add(planeXaxis.MulScalar(tri[1].X).Add(planeYaxis.MulScalar(tri[1].Y)))
		v2 := origin.Add(planeXaxis.MulScalar(tri[2].X).Add(planeYaxis.MulScalar(tri[2].Y)))
		result = append(result, NewTriangle3(v0, v1, v2))
	}

	return result, nil
}

// Construct a new, simple, polygon that consists of p and holes.
// We assume vertices in p are in CCW order and all holes in CW order, just as
// they naturally occur when tracing polygon outlines from our mesh.
// See https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf
func mergeHoles(p *Polygon, holes []*Polygon) (*Polygon, error) {
	result := NewPolygon()
	result.AddV2Set(p.Vertices())

	// Determine the maximum X value of hole vertices
	maxXVertexbyPolygon := make(map[*Polygon]int)
	for _, h := range holes {
		maxX := -math.MaxFloat64
		maxIdx := -1
		for i, v := range h.Vertices() {
			if v.X > maxX {
				maxX = v.X
				maxIdx = i
			}
		}
		maxXVertexbyPolygon[h] = maxIdx
	}

	// iterate through our holes in descending max-X order (= outer to inner) and merge
	for len(maxXVertexbyPolygon) > 0 {
		var hole *Polygon
		maxIdx := -1
		maxX := -math.MaxFloat64
		for h, idx := range maxXVertexbyPolygon {
			v := h.Vertices()[idx]
			if v.X > maxX {
				maxX = v.X
				hole = h
				maxIdx = idx
			}
		}
		delete(maxXVertexbyPolygon, hole)

		// reorder the hole's vertices such that the maximum X vertice comes first
		holeVs := hole.Vertices()
		holeVs = append(holeVs[maxIdx:], holeVs[:maxIdx]...)

		// find a mutually visible vertex in the outer polygon
		outerV, vIdx, err := findVisibleVertex(result, holeVs[0])
		if err != nil {
			return nil, err
		}

		// merge the hole in immediately after the visible vertex
		result.AddV2At(vIdx + 1, holeVs...)

		// close the inner hole and connect back to the outer polygon
		result.AddV2At(vIdx + len(holeVs) + 1, holeVs[0])
		result.AddV2At(vIdx + len(holeVs) + 2, outerV)
	}

	return result, nil
}

// returns a vertice (and its index) in poly that is visible from m
func findVisibleVertex(poly *Polygon, m V2) (V2, int, error) {
	// cast a ray in x direction, find the closest polygon segment that it intersects
	verts := poly.Vertices()

	closestIntersectionX := math.MaxFloat64
	closestIntersectionIdx := -1
	for i, v1 := range verts {
		j := (i + 1) % len(verts)
		v2 := verts[j]

		// vertices are in CCW order, see if this segment crosses in Y-direction
		if v1.Y <= m.Y && v2.Y >= m.Y {
			// make sure that one of the vertices is to the right of m
			if v1.X > m.X || v2.X > m.X {

				// compute intersection. See https://stackoverflow.com/questions/563198
				r := V2{1, 0}
				s := v2.Sub(v1)

				a := r.Cross(s)
				b := v1.Sub(m).Cross(r)

				if Abs(a) < epsilon && Abs(b) < epsilon {
					// Both are colinear. Given that m should be a vertex of a hole in poly,
					// this can only happen if both v1 and v2 are to the right of m.
					// We will then return the closer of the two.
					if v1.X < v2.X && v1.X > m.X {
						if v1.X < closestIntersectionX {
							closestIntersectionX = v1.X
							closestIntersectionIdx = i
						}
					} else if v2.X < v1.X && v2.X > m.X {
						if v2.X < closestIntersectionX {
							closestIntersectionX = v2.X
							closestIntersectionIdx = j
						}
					} else {
						// This shouldn't happen if our polygons are well-formed.
						return V2{}, 0, fmt.Errorf(
							"Collinear vertices %f and %f on both sides of point %f",
							v1, v2, m,
						)
					}
				} else if Abs(a) < epsilon && Abs(b) > epsilon {
					// Parallel and non-intersecting. Given our prior check that the segment
					// crosses in Y-direction, this shouldn't happen.
					return V2{}, 0, fmt.Errorf(
						"Segment between %f and %f doesn't intersect with %f",
						v1, v2, m,
					)
				} else if Abs(a) > epsilon {
					t := v1.Sub(m).Cross(s) / a
					u := b / a

					// compute the intersection point (two ways to be sure)
					i1 := m.Add(r.MulScalar(t))
					i2 := v1.Add(s.MulScalar(u))
					if i1.Sub(i2).Length() > epsilon {
						return V2{}, 0, fmt.Errorf(
							"Intersections %f and %f are different for vertices %f %f and point %f",
							i1, i2, v1, v2, m,
						)
					}
					if i1.X < closestIntersectionX {
						closestIntersectionX = i1.X
						closestIntersectionIdx = i
					}
				}

			}
		}
	}

	if closestIntersectionIdx < 0 {
		return V2{}, 0, fmt.Errorf("Couldn't find any visible vertices for %f", m)
	}
	if closestIntersectionX < m.X {
		return V2{}, 0, fmt.Errorf("Intersection %f is left of %f", closestIntersectionIdx, m)
	}

	p := verts[closestIntersectionIdx]
	i := V2{closestIntersectionX, m.Y}
	// Check if any points lie inside the triangle (m, p, i)
	var insidePoints []int
	for idx, x := range verts {
		if idx != closestIntersectionIdx {
			if triangleContains(m, p, i, x) {
				insidePoints = append(insidePoints, idx)
			}
		}
	}

	if len(insidePoints) == 0 {
		// no points lie inside the triangle, this implies p is visible
		return p, closestIntersectionIdx, nil
	} else if len(insidePoints) == 1 {
		// a single point lies inside the triangle, return that.
		return verts[insidePoints[0]], insidePoints[0], nil
	} else {
		// find the inside point with minimal angle to (m, i)
		toI := i.Sub(m)
		minAngle := math.MaxFloat64
		minAngleIdx := -1
		for _, idx := range insidePoints {
			v := verts[idx]
			toV := v.Sub(m)
			angle := math.Acos(toI.Dot(toV))
			if angle < minAngle {
				minAngle = angle
				minAngleIdx = idx
			}
		}
		return verts[minAngleIdx], minAngleIdx, nil
	}

	return V2{}, 0, nil
}

func triangleContains(a, b, c, p V2) bool {
    r := 1 / (-b.Y * c.X + a.Y * (-b.X + c.X) + a.X * (b.Y - c.Y) + b.X * c.Y)
    u := (a.Y * c.X - a.X * c.Y + (c.Y - a.Y) * p.X + (a.X - c.X) * p.Y)  *  r
    v := (a.X * b.Y - a.Y * b.X + (a.Y - b.Y) * p.X + (b.X - a.X) * p.Y) * r
    if u < -epsilon || u > 1 + epsilon || v < -epsilon || v > 1 + epsilon || u+v > 1 + epsilon {
        return false
    }
    return true
}

func isReflex(a, b, c V2) bool {
	return (b.X-a.X)*(c.Y-b.Y)-(c.X-b.X)*(b.Y-a.Y) < 0
}

func triangulate(p *Polygon) ([]Triangle2, error) {
	// Ear clipping for simplicity. Consider replacing with Seidel's algorithm.
	verts := p.Vertices()
	numVs := len(verts)

	reflexVerts := make([]bool, numVs)
	for i, v := range verts {
		iPrev := (i - 1 + numVs) % numVs
		iNext := (i + 1) % numVs

		vPrev := verts[iPrev]
		vNext := verts[iNext]
		if isReflex(vPrev, v, vNext) {
			reflexVerts[i] = true
		}
	}

	var result []Triangle2

	for len(verts) >= 3 {
		toRemove := -1
		for i, v := range verts {
			iPrev := (i - 1 + numVs) % numVs
			iNext := (i + 1) % numVs

			vPrev := verts[iPrev]
			vNext := verts[iNext]

			if !reflexVerts[i] {
				// see if any other point lies inside this triangle
				pointInside := false
				for j := 0; j < len(verts) - 3; j++ {
					checkIdx := (i + 2 + j) % len(verts)
					if reflexVerts[checkIdx] && triangleContains(vPrev, v, vNext, verts[checkIdx]) {
						pointInside = true
						break
					}
				}

				if !pointInside {
					result = append(result, Triangle2{vPrev, v, vNext})
					toRemove = i
					break
				}

			}
		}
		if toRemove >= 0 {
			verts = append(verts[:toRemove], verts[toRemove+1:]...)
			reflexVerts = append(reflexVerts[:toRemove], reflexVerts[toRemove+1:]...)

			// we need to recompute the reflex status for toRemove - 1 and toRemove
			numVs--
			idx0 := (toRemove - 2 + numVs) % numVs
			idx1 := (toRemove - 1 + numVs) % numVs
			idx2 := (toRemove + numVs) % numVs
			idx3 := (toRemove + 1 + numVs) % numVs

			reflexVerts[idx1] = isReflex(verts[idx0], verts[idx1], verts[idx2])
			reflexVerts[idx2] = isReflex(verts[idx1], verts[idx2], verts[idx3])
		} else {
			return nil, fmt.Errorf("Could not find ear. Vertices: %f", verts)
		}
	}
	return result, nil
}
