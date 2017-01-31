//-----------------------------------------------------------------------------
/*


 */
//-----------------------------------------------------------------------------

package main

import (
	"math"

	. "github.com/deadsy/sdfx/sdf"
)

//-----------------------------------------------------------------------------
// overall build controls

const MM_PER_INCH = 25.4
const SCALE = 1.0 / 0.98 // 2% Al shrinkage
const core_print = true  // add the core print to the wheel
const pie_print = true   // create a 1/n pie segment (n = number of webs)

//-----------------------------------------------------------------------------

// dimension scaling
func dim(x float64) float64 {
	return SCALE * x
}

//-----------------------------------------------------------------------------

// draft angles
var draft_angle = DtoR(4.0)       // standard overall draft
var core_draft_angle = DtoR(10.0) // draft angle for the core print

// nominal size values (mm)
var wheel_diameter = dim(MM_PER_INCH * 8.0) // total wheel diameter
var hub_diameter = dim(40.0)                // base diameter of central shaft hub
var hub_height = dim(53.0)                  // height of cental shaft hub
var shaft_diameter = dim(21.0)              // 1" target size - reduced for machining allowance
var shaft_length = dim(45.0)                // length of shaft bore
var wall_height = dim(35.0)                 // height of wheel side walls
var wall_thickness = dim(4.0)               // base thickness of outer wheel walls
var plate_thickness = dim(7.0)              // thickness of wheel top plate
var web_width = dim(4.0)                    // base thickness of reinforcing webs
var web_height = dim(25.0)                  // height of reinforcing webs
var core_height = dim(15.0)                 // height of core print
var number_of_webs = 6                      // number of reinforcing webs

// derived values
var wheel_radius = wheel_diameter / 2
var hub_radius = hub_diameter / 2
var shaft_radius = shaft_diameter / 2
var web_length = wheel_radius - (wall_thickness / 2) - shaft_radius

//-----------------------------------------------------------------------------

// build 2d wheel profile
func wheel_profile() SDF2 {

	draft0 := (hub_height - plate_thickness) * math.Tan(draft_angle)
	draft1 := (wall_height - plate_thickness) * math.Tan(draft_angle)
	draft2 := wall_height * math.Tan(draft_angle)
	draft3 := core_height * math.Tan(core_draft_angle)

	s := NewSmoother(false)

	if core_print {
		s.Add(V2{0, 0})
		s.Add(V2{wheel_radius + draft2, 0})
		s.AddSmooth(V2{wheel_radius, wall_height}, 5, 1.0)
		s.AddSmooth(V2{wheel_radius - wall_thickness, wall_height}, 5, 1.0)
		s.AddSmooth(V2{wheel_radius - wall_thickness - draft1, plate_thickness}, 5, 2.0)
		s.AddSmooth(V2{hub_radius + draft0, plate_thickness}, 5, 2.0)
		s.AddSmooth(V2{hub_radius, hub_height}, 5, 2.0)
		s.Add(V2{shaft_radius, hub_height})
		s.Add(V2{shaft_radius - draft3, hub_height + core_height})
		s.Add(V2{0, hub_height + core_height})
	} else {
		s.Add(V2{0, 0})
		s.Add(V2{wheel_radius + draft2, 0})
		s.AddSmooth(V2{wheel_radius, wall_height}, 5, 1.0)
		s.AddSmooth(V2{wheel_radius - wall_thickness, wall_height}, 5, 1.0)
		s.AddSmooth(V2{wheel_radius - wall_thickness - draft1, plate_thickness}, 5, 2.0)
		s.AddSmooth(V2{hub_radius + draft0, plate_thickness}, 5, 2.0)
		s.AddSmooth(V2{hub_radius, hub_height}, 5, 2.0)
		s.Add(V2{shaft_radius, hub_height})
		s.Add(V2{shaft_radius, hub_height - shaft_length})
		s.Add(V2{0, hub_height - shaft_length})
	}
	s.Smooth()

	RenderDXF("wheel.dxf", s.Vertices())
	return NewPolySDF2(s.Vertices())
}

//-----------------------------------------------------------------------------

// build 2d web profile
func web_profile() SDF2 {

	draft := web_height * math.Tan(draft_angle)
	x1 := web_width + draft
	x2 := web_width

	s := NewSmoother(false)
	s.Add(V2{-x1, 0})
	s.AddSmooth(V2{-x2, web_height}, 3, 1.0)
	s.AddSmooth(V2{x2, web_height}, 3, 1.0)
	s.Add(V2{x1, 0})
	s.Smooth()

	RenderDXF("web.dxf", s.Vertices())
	return NewPolySDF2(s.Vertices())
}

//-----------------------------------------------------------------------------

// build 2d core profile
func core_profile() SDF2 {

	draft := core_height * math.Tan(core_draft_angle)

	s := NewSmoother(false)
	s.Add(V2{0, 0})
	s.Add(V2{shaft_radius - draft, 0})
	s.Add(V2{shaft_radius, core_height})
	s.AddSmooth(V2{shaft_radius, core_height + shaft_length}, 3, 2.0)
	s.Add(V2{0, core_height + shaft_length})
	s.Smooth()

	RenderDXF("core.dxf", s.Vertices())
	return NewPolySDF2(s.Vertices())
}

//-----------------------------------------------------------------------------

func wheel() {
	//web := web_profile()
	//core := core_profile()

	s0 := wheel_profile()
	s1 := NewSorSDF3(s0)

	//s2 := web_profile()
	//s3 := NewExtrudeSDF3(s2, web_length)

	RenderSTL(s1)
}

//-----------------------------------------------------------------------------