//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

package sdf

import (
	"fmt"
	"math"
	"testing"
)

//-----------------------------------------------------------------------------

const TOLERANCE = 1e-9

//-----------------------------------------------------------------------------

func Test_Determinant(t *testing.T) {
	m := M33{2, 3, 1, -1, -6, 7, 4, 5, -1}
	if m.Determinant() != 42 {
		t.Error("FAIL")
	}
}

//-----------------------------------------------------------------------------

func Test_Inverse(t *testing.T) {
	a := M33{2, 1, 1, 3, 2, 1, 2, 1, 2}
	a_inv := M33{3, -1, -1, -4, 2, 1, -1, 0, 1}
	if a.Inverse().Equals(a_inv, TOLERANCE) == false {
		t.Error("FAIL")
	}
	if a.Mul(a_inv).Equals(Identity2d(), TOLERANCE) == false {
		t.Error("FAIL")
	}

	for i := 0; i < 100; i++ {
		a = RandomM33(-5, 5)
		a_inv = a.Inverse()
		if a.Mul(a_inv).Equals(Identity2d(), TOLERANCE) == false {
			t.Error("FAIL")
		}
	}

	for i := 0; i < 100; i++ {
		b := RandomM44(-1, 1)
		b_inv := b.Inverse()
		if b.Mul(b_inv).Equals(Identity3d(), TOLERANCE) == false {
			t.Error("FAIL")
		}
	}

	for i := 0; i < 100; i++ {
		c := RandomM22(-7, 7)
		c_inv := c.Inverse()
		if c.Mul(c_inv).Equals(Identity(), TOLERANCE) == false {
			t.Error("FAIL")
		}
	}
}

//-----------------------------------------------------------------------------

func Test_MulBox(t *testing.T) {

	// 2D boxes
	b2d := Box2{V2{-1, -1}, V2{1, 1}}
	for i := 0; i < 100; i++ {
		v := RandomV2(-5, 5)
		// translating
		m0 := Translate2d(v)
		m1 := Translate2d(v.Negate())
		b1 := m0.MulBox(b2d)
		b2 := m1.MulBox(b1)
		if b2d.Equals(b2, TOLERANCE) == false {
			t.Error("FAIL")
		}
		// scaling
		m0 = Scale2d(v)
		m1 = Scale2d(V2{1 / v.X, 1 / v.Y})
		b1 = m0.MulBox(b2d)
		b2 = m1.MulBox(b1)
		if b2d.Equals(b2, TOLERANCE) == false {
			t.Error("FAIL")
		}
	}

	// 3D boxes
	b3d := Box3{V3{-1, -1, -1}, V3{1, 1, 1}}
	for i := 0; i < 100; i++ {
		v := RandomV3(-5, 5)
		// translating
		m0 := Translate3d(v)
		m1 := Translate3d(v.Negate())
		b1 := m0.MulBox(b3d)
		b2 := m1.MulBox(b1)
		if b3d.Equals(b2, TOLERANCE) == false {
			t.Error("FAIL")
		}
		// scaling
		m0 = Scale3d(v)
		m1 = Scale3d(V3{1 / v.X, 1 / v.Y, 1 / v.Z})
		b1 = m0.MulBox(b3d)
		b2 = m1.MulBox(b1)
		if b3d.Equals(b2, TOLERANCE) == false {
			t.Error("FAIL")
		}
	}
}

//-----------------------------------------------------------------------------

func Test_ScaleBox(t *testing.T) {
	b0 := Box3{V3{-1, -1, -1}, V3{1, 1, 1}}
	b1 := Box3{V3{-2, -2, -2}, V3{2, 2, 2}}
	b2 := NewBox3(b0.Center(), b0.Size().MulScalar(2))
	if b1.Equals(b2, TOLERANCE) == false {
		t.Error("FAIL")
	}
}

//-----------------------------------------------------------------------------

func Test_Line(t *testing.T) {

	l := NewLine2_PP(V2{0, 1}, V2{0, 2})
	points := []struct {
		p V2
		d float64
	}{
		{V2{0, 1}, 0},
		{V2{0, 2}, 0},
		{V2{0, 1.5}, 0},
		{V2{0, 0}, 1},
		{V2{0, 3}, 1},
		{V2{0.5, 1.1}, 0.5},
		{V2{-0.5, 1.1}, -0.5},
		{V2{0.1, 1.98}, 0.1},
		{V2{-0.1, 1.98}, -0.1},
		{V2{3, 6}, 5},
		{V2{-3, 6}, -5},
		{V2{3, -3}, 5},
		{V2{-3, -3}, -5},
	}
	for _, p := range points {
		d := l.Distance(p.p)
		if Abs(d-p.d) > TOLERANCE {
			fmt.Printf("%+v %f (expected) %f (actual)\n", p.p, p.d, d)
			t.Error("FAIL")
		}
	}

	line_tests := []struct {
		p0, v0 V2
		p1, v1 V2
		t0     float64
		t1     float64
		err    string
	}{
		{V2{0, 0}, V2{0, 1}, V2{1, 0}, V2{-1, 0}, 0, 1, ""},
		{V2{0, 0}, V2{0, 1}, V2{1, 1}, V2{0, 1}, 0, 1, "zero/many"},
		{V2{0, 0}, V2{0, 1}, V2{0, 0}, V2{0, 1}, 0, 1, "zero/many"},
		{V2{0, 0}, V2{1, 1}, V2{0, 10}, V2{1, -1}, 5 * math.Sqrt(2), 5 * math.Sqrt(2), ""},
		{V2{0, 0}, V2{1, 1}, V2{10, 0}, V2{0, 1}, 10 * math.Sqrt(2), 10, ""},
	}
	for _, test := range line_tests {
		l0 := NewLine2_PV(test.p0, test.v0)
		l1 := NewLine2_PV(test.p1, test.v1)
		t0, t1, err := l0.Intersect(l1)
		if err != nil {
			if err.Error() != test.err {
				fmt.Printf("l0: %+v\n", l0)
				fmt.Printf("l1: %+v\n", l1)
				fmt.Printf("error: %s\n", err)
				t.Error("FAIL")
			}
		} else {
			if Abs(test.t0-t0) > TOLERANCE || Abs(test.t1-t1) > TOLERANCE {
				fmt.Printf("l0: %+v\n", l0)
				fmt.Printf("l1: %+v\n", l1)
				fmt.Printf("%f %f (expected) %f %f (actual)\n", test.t0, test.t1, t0, t1)
				t.Error("FAIL")
			}
		}
	}

	for i := 0; i < 10000; i++ {
		l0 := NewLine2_PV(RandomV2(-10, 10), RandomV2(-10, 10))
		l1 := NewLine2_PP(RandomV2(-10, 10), RandomV2(-10, 10))
		t0, t1, err := l0.Intersect(l1)
		if err != nil {
			continue
		}
		i0 := l0.Position(t0)
		i1 := l1.Position(t1)
		if !i0.Equals(i1, TOLERANCE) {
			fmt.Printf("l0: %+v\n", l0)
			fmt.Printf("l1: %+v\n", l1)
			t.Error("FAIL")
		}
	}
}

//-----------------------------------------------------------------------------

func Test_Polygon1(t *testing.T) {
	s := Polygon2D([]V2{V2{0, 0}, V2{1, 0}, V2{0, 1}})
	b := s.BoundingBox()
	b0 := Box2{V2{0, 0}, V2{1, 1}}
	if b.Equals(b0, TOLERANCE) == false {
		t.Error("FAIL")
	}

	s = Polygon2D([]V2{V2{0, -2}, V2{1, 1}, V2{-2, 2}})
	b = s.BoundingBox()
	b0 = Box2{V2{-2, -2}, V2{1, 2}}
	if b.Equals(b0, TOLERANCE) == false {
		t.Error("FAIL")
	}

	points := []V2{
		V2{0, -1},
		V2{1, 1},
		V2{-1, 1},
	}

	s = Polygon2D(points)

	b = s.BoundingBox()
	b0 = Box2{V2{-1, -1}, V2{1, 1}}
	if b.Equals(b0, TOLERANCE) == false {
		t.Error("FAIL")
	}

	test_points := []struct {
		p V2
		d float64
	}{
		{V2{0, -1}, 0},
		{V2{1, 1}, 0},
		{V2{-1, 1}, 0},
		{V2{0, 1}, 0},
		{V2{0, 2}, 1},
		{V2{0, -2}, 1},
		{V2{1, 0}, 1 / math.Sqrt(5)},
		{V2{-1, 0}, 1 / math.Sqrt(5)},
		{V2{0, 0}, -1 / math.Sqrt(5)},
		{V2{3, 0}, math.Sqrt(5)},
		{V2{-3, 0}, math.Sqrt(5)},
	}

	for _, p := range test_points {
		d := s.Evaluate(p.p)
		if d != p.d {
			fmt.Printf("%+v %f (expected) %f (actual)\n", p.p, p.d, d)
			t.Error("FAIL")
		}
	}
}

//-----------------------------------------------------------------------------

func Test_Polygon2(t *testing.T) {
	k := 1.2

	s0 := Polygon2D([]V2{V2{k, -k}, V2{k, k}, V2{-k, k}, V2{-k, -k}})
	s0 = Transform2D(s0, Translate2d(V2{0.8, 0}))

	s1 := Box2D(V2{2 * k, 2 * k}, 0)
	s1 = Transform2D(s1, Translate2d(V2{0.8, 0}))

	for i := 0; i < 10000; i++ {
		p := RandomV2(-10*k, 10*k)
		if Abs(s0.Evaluate(p)-s1.Evaluate(p)) > TOLERANCE {
			t.Error("FAIL")
		}
	}
}

//-----------------------------------------------------------------------------

func Test_Polygon3(t *testing.T) {

	// size
	a := 1.4
	b := 2.2
	// rotation
	theta := -15.0
	c := math.Cos(DtoR(theta))
	s := math.Sin(DtoR(theta))
	// translate
	j := -1.0
	k := 2.0

	s1 := Box2D(V2{2 * a, 2 * b}, 0)
	s1 = Transform2D(s1, Rotate2d(DtoR(theta)))
	s1 = Transform2D(s1, Translate2d(V2{j, k}))

	points := []V2{
		V2{j + c*a - s*b, k + s*a + c*b},
		V2{j - c*a - s*b, k - s*a + c*b},
		V2{j - c*a + s*b, k - s*a - c*b},
		V2{j + c*a + s*b, k + s*a - c*b},
	}

	s0 := Polygon2D(points)

	for i := 0; i < 1000; i++ {
		p := RandomV2(-5*b, 5*b)
		if Abs(s0.Evaluate(p)-s1.Evaluate(p)) > TOLERANCE {
			t.Error("FAIL")
		}
	}
}

//-----------------------------------------------------------------------------

func Test_ArraySDF2(t *testing.T) {
	r := 0.5
	s := Circle2D(r)
	bb := s.BoundingBox()
	if bb.Min.Equals(V2{-r, -r}, TOLERANCE) == false {
		t.Error("FAIL")
	}
	if bb.Max.Equals(V2{r, r}, TOLERANCE) == false {
		t.Error("FAIL")
	}

	j := 3
	k := 4
	dx := 2.0
	dy := 7.0
	sa := Array2D(s, V2i{j, k}, V2{dx, dy})
	sa_bb := sa.BoundingBox()
	if sa_bb.Min.Equals(V2{-r, -r}, TOLERANCE) == false {
		t.Error("FAIL")
	}
	if sa_bb.Max.Equals(V2{r + (float64(j-1) * dx), r + (float64(k-1) * dy)}, TOLERANCE) == false {
		t.Error("FAIL")
	}

	j = 7
	k = 4
	dx = -3.0
	dy = -5.0
	sa = Array2D(s, V2i{j, k}, V2{dx, dy})
	sa_bb = sa.BoundingBox()
	if sa_bb.Min.Equals(V2{-r + (float64(j-1) * dx), -r + (float64(k-1) * dy)}, TOLERANCE) == false {
		t.Error("FAIL")
	}
	if sa_bb.Max.Equals(V2{r, r}, TOLERANCE) == false {
		t.Error("FAIL")
	}

	j = 6
	k = 8
	dx = 5.0
	dy = -3.0
	sa = Array2D(s, V2i{j, k}, V2{dx, dy})
	sa_bb = sa.BoundingBox()
	if sa_bb.Min.Equals(V2{-r, -r + (float64(k-1) * dy)}, TOLERANCE) == false {
		t.Error("FAIL")
	}
	if sa_bb.Max.Equals(V2{r + (float64(j-1) * dx), r}, TOLERANCE) == false {
		t.Error("FAIL")
	}

	j = 9
	k = 1
	dx = -0.5
	dy = 6.5
	sa = Array2D(s, V2i{j, k}, V2{dx, dy})
	sa_bb = sa.BoundingBox()
	if sa_bb.Min.Equals(V2{-r + (float64(j-1) * dx), -r}, TOLERANCE) == false {
		t.Error("FAIL")
	}
	if sa_bb.Max.Equals(V2{r, r + (float64(k-1) * dy)}, TOLERANCE) == false {
		t.Error("FAIL")
	}
}

//-----------------------------------------------------------------------------

func Test_Rotation2d(t *testing.T) {
	r := Rotate2d(DtoR(90))
	v := V2{1, 0}
	v = r.MulPosition(v)
	if v.Equals(V2{0, 1}, TOLERANCE) == false {
		t.Error("FAIL")
	}
}

func Test_Rotation3d(t *testing.T) {
	r := Rotate3d(V3{0, 0, 1}, DtoR(90))
	v := V3{1, 0, 0}
	v = r.MulPosition(v)
	if v.Equals(V3{0, 1, 0}, TOLERANCE) == false {
		t.Error("FAIL")
	}
}

//-----------------------------------------------------------------------------

func Test_TriDiagonal(t *testing.T) {
	n := 5
	m := make([]V3, n)
	for i := 0; i < n; i++ {
		m[i].X = 1
		m[i].Y = 4
		m[i].Z = 1
	}
	m[0].X = 0
	m[0].Y = 2
	m[n-1].Y = 2
	m[n-1].Z = 0

	d := []float64{0, 1, 2, 3, 4}
	x0 := []float64{-1.0 / 12.0, 1.0 / 6.0, 5.0 / 12.0, 1.0 / 6.0, 23.0 / 12.0}
	x := TriDiagonal(m, d)
	for i := 0; i < n; i++ {
		if Abs(x[i]-x0[i]) > TOLERANCE {
			t.Error("FAIL")
		}
	}

	d = []float64{10, 20, 30, 40, 50}
	x0 = []float64{15.0 / 4.0, 5.0 / 2.0, 25.0 / 4.0, 5.0 / 2.0, 95.0 / 4.0}
	x = TriDiagonal(m, d)
	for i := 0; i < n; i++ {
		if Abs(x[i]-x0[i]) > TOLERANCE {
			t.Error("FAIL")
		}
	}

	m[0] = V3{0, 1, 2}
	m[1] = V3{3, 4, 5}
	m[2] = V3{6, 7, 8}
	m[3] = V3{9, 10, 11}
	m[4] = V3{12, 13, 0}
	d = []float64{-10, -20, -30, 40, 50}
	x0 = []float64{60.0 / 49.0, -275.0 / 49.0, -12.0 / 49.0, 33.0 / 49.0, 158.0 / 49.0}
	x = TriDiagonal(m, d)
	for i := 0; i < n; i++ {
		if Abs(x[i]-x0[i]) > TOLERANCE {
			t.Error("FAIL")
		}
	}

}

//-----------------------------------------------------------------------------

func Test_CubicSpline(t *testing.T) {

	data := []V2{
		V2{-1.5, -1.2},
		V2{-0.2, 0},
		V2{1, 0.5},
		V2{5, 1},
		V2{10, 2.2},
		V2{12, 3.2},
		V2{16, -1.2},
		V2{18, -3.2},
	}
	cs := NewCubicSpline(data)
	n := len(cs.spline)

	if n != len(data)-1 {
		t.Error("FAIL")
	}

	// check for agreement with the input data
	for _, v := range data {
		y := cs.Function(v.X)
		if Abs(y-v.Y) > TOLERANCE {
			t.Error("FAIL")
		}
	}

	// check for agreement with the input data
	for i, s := range cs.spline {
		// Check the x0 value
		x0 := s.p0.X
		if Abs(x0-data[i].X) > TOLERANCE {
			t.Error("FAIL")
		}
		// Check the x1 value
		x1 := s.p1.X
		if Abs(x1-data[i+1].X) > TOLERANCE {
			t.Error("FAIL")
		}
		// Check the y0 value
		if Abs(s.Function(s.XtoT(x0))-data[i].Y) > TOLERANCE {
			t.Error("FAIL")
		}
		// Check the y1 value
		if Abs(s.Function(s.XtoT(x1))-data[i+1].Y) > TOLERANCE {
			t.Error("FAIL")
		}
	}

	// check for continuity of 1st,2nd derivatives
	s := cs.spline[0]
	yd1 := s.b + 2*s.c + 3*s.d
	ydd1 := 2*s.c + 6*s.d
	for i := 1; i < n; i++ {
		s = cs.spline[i]
		yd0 := s.b
		ydd0 := 2 * s.c
		if Abs(yd1-yd0) > TOLERANCE {
			t.Error("FAIL")
		}
		if Abs(ydd1-ydd0) > TOLERANCE {
			t.Error("FAIL")
		}
		yd1 = s.b + 2*s.c + 3*s.d
		ydd1 = 2*s.c + 6*s.d
	}

	// check for 2nd derivative == 0 at endpoints
	s = cs.spline[0]
	ydd := 2 * s.c
	if Abs(ydd) > TOLERANCE {
		t.Error("FAIL")
	}
	s = cs.spline[n-1]
	ydd = 2*s.c + 6*s.d
	if Abs(ydd) > TOLERANCE {
		t.Error("FAIL")
	}

}

//-----------------------------------------------------------------------------

func Test_Quadratic(t *testing.T) {

	x, rc := quadratic(4, 2, 1)
	if x != nil || rc != ZERO_SOLN {
		t.Error("FAIL")
	}

	x, rc = quadratic(0, 0, 1)
	if x != nil || rc != ZERO_SOLN {
		t.Error("FAIL")
	}

	x, rc = quadratic(0, 2, -4)
	if x[0] != 2 || rc != ONE_SOLN {
		t.Error("FAIL")
	}

	x, rc = quadratic(1, -5, 6)
	if x[0] != 3 || x[1] != 2 || rc != TWO_SOLN {
		t.Error("FAIL")
	}

	x, rc = quadratic(0, 0, 0)
	if x != nil || rc != INF_SOLN {
		t.Error("FAIL")
	}
}

//-----------------------------------------------------------------------------
