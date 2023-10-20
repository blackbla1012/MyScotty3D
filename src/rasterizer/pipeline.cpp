// clang-format off
#include "pipeline.h"

#include <iostream>

#include "../lib/log.h"
#include "../lib/mathlib.h"
#include "framebuffer.h"
#include "sample_pattern.h"
template<PrimitiveType primitive_type, class Program, uint32_t flags>
void Pipeline<primitive_type, Program, flags>::run(std::vector<Vertex> const& vertices,
                                                   typename Program::Parameters const& parameters,
                                                   Framebuffer* framebuffer_) {
	// Framebuffer must be non-null:
	assert(framebuffer_);
	auto& framebuffer = *framebuffer_;

	// A1T7: sample loop
	// TODO: update this function to rasterize to *all* sample locations in the framebuffer.
	//  	 This will probably involve inserting a loop of the form:
	// 		 	std::vector< Vec3 > const &samples = framebuffer.sample_pattern.centers_and_weights;
	//      	for (uint32_t s = 0; s < samples.size(); ++s) { ... }
	//   	 around some subset of the code.
	// 		 You will also need to transform the input and output of the rasterize_* functions to
	// 	     account for the fact they deal with pixels centered at (0.5,0.5).

	std::vector<ShadedVertex> shaded_vertices;
	shaded_vertices.reserve(vertices.size());

	//--------------------------
	// shade vertices:
	for (auto const& v : vertices) {
		ShadedVertex sv;
		Program::shade_vertex(parameters, v.attributes, &sv.clip_position, &sv.attributes);
		shaded_vertices.emplace_back(sv);
	}

	//--------------------------
	// assemble + clip + homogeneous divide vertices:
	std::vector<ClippedVertex> clipped_vertices;

	// reserve some space to avoid reallocations later:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		// clipping lines can never produce more than one vertex per input vertex:
		clipped_vertices.reserve(shaded_vertices.size());
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		// clipping triangles can produce up to 8 vertices per input vertex:
		clipped_vertices.reserve(shaded_vertices.size() * 8);
	}
	// clang-format off

	//coefficients to map from clip coordinates to framebuffer (i.e., "viewport") coordinates:
	//x: [-1,1] -> [0,width]
	//y: [-1,1] -> [0,height]
	//z: [-1,1] -> [0,1] (OpenGL-style depth range)
	Vec3 const clip_to_fb_scale = Vec3{
		framebuffer.width / 2.0f,
		framebuffer.height / 2.0f,
		0.5f
	};
	Vec3 const clip_to_fb_offset = Vec3{
		0.5f * framebuffer.width,
		0.5f * framebuffer.height,
		0.5f
	};

	// helper used to put output of clipping functions into clipped_vertices:
	auto emit_vertex = [&](ShadedVertex const& sv) {
		ClippedVertex cv;
		float inv_w = 1.0f / sv.clip_position.w;
		cv.fb_position = clip_to_fb_scale * inv_w * sv.clip_position.xyz() + clip_to_fb_offset;
		cv.inv_w = inv_w;
		cv.attributes = sv.attributes;
		clipped_vertices.emplace_back(cv);
	};

	// actually do clipping:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		for (uint32_t i = 0; i + 1 < shaded_vertices.size(); i += 2) {
			clip_line(shaded_vertices[i], shaded_vertices[i + 1], emit_vertex);
		}
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		for (uint32_t i = 0; i + 2 < shaded_vertices.size(); i += 3) {
			clip_triangle(shaded_vertices[i], shaded_vertices[i + 1], shaded_vertices[i + 2], emit_vertex);
		}
	} else {
		static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
	}

	//--------------------------
	// rasterize primitives:

	// helper used to put output of rasterization functions into fragments:

	std::vector< Vec3 > const &samples = framebuffer.sample_pattern.centers_and_weights;
	for (uint32_t s = 0; s < samples.size(); ++s) {

		std::vector<Fragment> fragments;

		float xShift = samples[s].x - 0.5f;
		float yShift = samples[s].y - 0.5f;

		auto emit_fragment = [&](Fragment const& f) { 
			Fragment SampleFrag = f;
			SampleFrag.fb_position.x += xShift;
			SampleFrag.fb_position.y += yShift;
			fragments.emplace_back(SampleFrag); 
		};

		// actually do rasterization:
		if constexpr (primitive_type == PrimitiveType::Lines) {
			for (uint32_t i = 0; i + 1 < clipped_vertices.size(); i += 2) {
				clipped_vertices[i].fb_position.x -= xShift;
				clipped_vertices[i].fb_position.y -= yShift;
				clipped_vertices[i + 1].fb_position.x -= xShift;
				clipped_vertices[i + 1].fb_position.y -= yShift;
				rasterize_line(clipped_vertices[i], clipped_vertices[i + 1], emit_fragment);
				clipped_vertices[i].fb_position.x += xShift;
				clipped_vertices[i].fb_position.y += yShift;
				clipped_vertices[i + 1].fb_position.x += xShift;
				clipped_vertices[i + 1].fb_position.y += yShift;
			}
		} else if constexpr (primitive_type == PrimitiveType::Triangles) {
			for (uint32_t i = 0; i + 2 < clipped_vertices.size(); i += 3) {
				clipped_vertices[i].fb_position.x -= xShift;
				clipped_vertices[i].fb_position.y -= yShift;
				clipped_vertices[i + 1].fb_position.x -= xShift;
				clipped_vertices[i + 1].fb_position.y -= yShift;
				clipped_vertices[i + 2].fb_position.x -= xShift;
				clipped_vertices[i + 2].fb_position.y -= yShift;
				rasterize_triangle(clipped_vertices[i], clipped_vertices[i + 1], clipped_vertices[i + 2], emit_fragment);
				clipped_vertices[i].fb_position.x += xShift;
				clipped_vertices[i].fb_position.y += yShift;
				clipped_vertices[i + 1].fb_position.x += xShift;
				clipped_vertices[i + 1].fb_position.y += yShift;
				clipped_vertices[i + 2].fb_position.x += xShift;
				clipped_vertices[i + 2].fb_position.y += yShift;
			}
		} else {
			static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
		}

		//--------------------------
		// depth test + shade + blend fragments:
		uint32_t out_of_range = 0; // check if rasterization produced fragments outside framebuffer 
								// (indicates something is wrong with clipping)
		for (auto const& f : fragments) {

			// fragment location (in pixels):
			int32_t x = (int32_t)std::floor(f.fb_position.x);
			int32_t y = (int32_t)std::floor(f.fb_position.y);

			// if clipping is working properly, this condition shouldn't be needed;
			// however, it prevents crashes while you are working on your clipping functions,
			// so we suggest leaving it in place:
			if (x < 0 || (uint32_t)x >= framebuffer.width || 
				y < 0 || (uint32_t)y >= framebuffer.height) {
				++out_of_range;
				continue;
			}

			// local names that refer to destination sample in framebuffer:
			float& fb_depth = framebuffer.depth_at(x, y, s);
			Spectrum& fb_color = framebuffer.color_at(x, y, s);


			// depth test:
			if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Always) {
				// "Always" means the depth test always passes.
			} else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Never) {
				// "Never" means the depth test never passes.
				continue; //discard this fragment
			} else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Less) {
				// "Less" means the depth test passes when the new fragment has depth less than the stored depth.
				// A1T4: Depth_Less
				if(f.fb_position.z < fb_depth){
					fb_depth = f.fb_position.z;
				}
				else {continue;}
			} else {
				static_assert((flags & PipelineMask_Depth) <= Pipeline_Depth_Always, "Unknown depth test flag.");
			}

			// if depth test passes, and depth writes aren't disabled, write depth to depth buffer:
			if constexpr (!(flags & Pipeline_DepthWriteDisableBit)) {
				fb_depth = f.fb_position.z;
			}

			// shade fragment:
			ShadedFragment sf;
			sf.fb_position = f.fb_position;
			Program::shade_fragment(parameters, f.attributes, f.derivatives, &sf.color, &sf.opacity);

			// write color to framebuffer if color writes aren't disabled:
			if constexpr (!(flags & Pipeline_ColorWriteDisableBit)) {
				// blend fragment:
				if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Replace) {
					fb_color = sf.color;
				} else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Add) {
					fb_color += sf.color * sf.opacity; 
				} else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Over) {
					fb_color = sf.color * sf.opacity + fb_color * (1-sf.opacity); 
				} else {
					static_assert((flags & PipelineMask_Blend) <= Pipeline_Blend_Over, "Unknown blending flag.");
				}
			}
		}
		if (out_of_range > 0) {
			if constexpr (primitive_type == PrimitiveType::Lines) {
				warn("Produced %d fragments outside framebuffer; this indicates something is likely "
					"wrong with the clip_line function.",
					out_of_range);
			} else if constexpr (primitive_type == PrimitiveType::Triangles) {
				warn("Produced %d fragments outside framebuffer; this indicates something is likely "
					"wrong with the clip_triangle function.",
					out_of_range);
			}
		}
	}
}

// -------------------------------------------------------------------------
// clipping functions

// helper to interpolate between vertices:
template<PrimitiveType p, class P, uint32_t F>
auto Pipeline<p, P, F>::lerp(ShadedVertex const& a, ShadedVertex const& b, float t) -> ShadedVertex {
	ShadedVertex ret;
	ret.clip_position = (b.clip_position - a.clip_position) * t + a.clip_position;
	for (uint32_t i = 0; i < ret.attributes.size(); ++i) {
		ret.attributes[i] = (b.attributes[i] - a.attributes[i]) * t + a.attributes[i];
	}
	return ret;
}

/*
 * clip_line - clip line to portion with -w <= x,y,z <= w, emit vertices of clipped line (if non-empty)
 *  	va, vb: endpoints of line
 *  	emit_vertex: call to produce truncated line
 *
 * If clipping shortens the line, attributes of the shortened line should respect the pipeline's interpolation mode.
 * 
 * If no portion of the line remains after clipping, emit_vertex will not be called.
 *
 * The clipped line should have the same direction as the full line.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_line(ShadedVertex const& va, ShadedVertex const& vb,
                                      std::function<void(ShadedVertex const&)> const& emit_vertex) {
	// Determine portion of line over which:
	// 		pt = (b-a) * t + a
	//  	-pt.w <= pt.x <= pt.w
	//  	-pt.w <= pt.y <= pt.w
	//  	-pt.w <= pt.z <= pt.w
	// ... as a range [min_t, max_t]:

	float min_t = 0.0f;
	float max_t = 1.0f;

	// want to set range of t for a bunch of equations like:
	//    a.x + t * ba.x <= a.w + t * ba.w
	// so here's a helper:
	auto clip_range = [&min_t, &max_t](float l, float dl, float r, float dr) {
		// restrict range such that:
		// l + t * dl <= r + t * dr
		// re-arranging:
		//  l - r <= t * (dr - dl)
		if (dr == dl) {
			// want: l - r <= 0
			if (l - r > 0.0f) {
				// works for none of range, so make range empty:
				min_t = 1.0f;
				max_t = 0.0f;
			}
		} else if (dr > dl) {
			// since dr - dl is positive:
			// want: (l - r) / (dr - dl) <= t
			min_t = std::max(min_t, (l - r) / (dr - dl));
		} else { // dr < dl
			// since dr - dl is negative:
			// want: (l - r) / (dr - dl) >= t
			max_t = std::min(max_t, (l - r) / (dr - dl));
		}
	};

	// local names for clip positions and their difference:
	Vec4 const& a = va.clip_position;
	Vec4 const& b = vb.clip_position;
	Vec4 const ba = b - a;

	// -a.w - t * ba.w <= a.x + t * ba.x <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.x, ba.x);
	clip_range(a.x, ba.x, a.w, ba.w);
	// -a.w - t * ba.w <= a.y + t * ba.y <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.y, ba.y);
	clip_range(a.y, ba.y, a.w, ba.w);
	// -a.w - t * ba.w <= a.z + t * ba.z <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.z, ba.z);
	clip_range(a.z, ba.z, a.w, ba.w);

	if (min_t < max_t) {
		if (min_t == 0.0f) {
			emit_vertex(va);
		} else {
			ShadedVertex out = lerp(va, vb, min_t);
			// don't interpolate attributes if in flat shading mode:
			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
				out.attributes = va.attributes;
			}
			emit_vertex(out);
		}
		if (max_t == 1.0f) {
			emit_vertex(vb);
		} else {
			ShadedVertex out = lerp(va, vb, max_t);
			// don't interpolate attributes if in flat shading mode:
			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
				out.attributes = va.attributes;
			}
			emit_vertex(out);
		}
	}
}

/*
 * clip_triangle - clip triangle to portion with -w <= x,y,z <= w, emit resulting shape as triangles (if non-empty)
 *  	va, vb, vc: vertices of triangle
 *  	emit_vertex: call to produce clipped triangles (three calls per triangle)
 *
 * If clipping truncates the triangle, attributes of the new vertices should respect the pipeline's interpolation mode.
 * 
 * If no portion of the triangle remains after clipping, emit_vertex will not be called.
 *
 * The clipped triangle(s) should have the same winding order as the full triangle.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_triangle(
	ShadedVertex const& va, ShadedVertex const& vb, ShadedVertex const& vc,
	std::function<void(ShadedVertex const&)> const& emit_vertex) {
	// A1EC: clip_triangle
	// TODO: correct code!
	emit_vertex(va);
	emit_vertex(vb);
	emit_vertex(vc);
}

// -------------------------------------------------------------------------
// rasterization functions

/*
 * rasterize_line:
 * calls emit_fragment( frag ) for every pixel "covered" by the line (va.fb_position.xy, vb.fb_position.xy).
 *
 *    a pixel (x,y) is "covered" by the line if it exits the inscribed diamond:
 * 
 *        (x+0.5,y+1)
 *        /        \
 *    (x,y+0.5)  (x+1,y+0.5)
 *        \        /
 *         (x+0.5,y)
 *
 *    to avoid ambiguity, we consider diamonds to contain their left and bottom points
 *    but not their top and right points. 
 * 
 * 	  since 45 degree lines breaks this rule, our rule in general is to rasterize the line as if its
 *    endpoints va and vb were at va + (e, e^2) and vb + (e, e^2) where no smaller nonzero e produces 
 *    a different rasterization result. 
 *    We will not explicitly check for 45 degree lines along the diamond edges (this will be extra credit),
 *    but you should be able to handle 45 degree lines in every other case (such as starting from pixel centers)
 *
 * for each such diamond, pass Fragment frag to emit_fragment, with:
 *  - frag.fb_position.xy set to the center (x+0.5,y+0.5)
 *  - frag.fb_position.z interpolated linearly between va.fb_position.z and vb.fb_position.z
 *  - frag.attributes set to va.attributes (line will only be used in Interp_Flat mode)
 *  - frag.derivatives set to all (0,0)
 *
 * when interpolating the depth (z) for the fragments, you may use any depth the line takes within the pixel
 * (i.e., you don't need to interpolate to, say, the closest point to the pixel center)
 *
 * If you wish to work in fixed point, check framebuffer.h for useful information about the framebuffer's dimensions.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_line(
	ClippedVertex const& va, ClippedVertex const& vb,
	std::function<void(Fragment const&)> const& emit_fragment) {
	if constexpr ((flags & PipelineMask_Interp) != Pipeline_Interp_Flat) {
		assert(0 && "rasterize_line should only be invoked in flat interpolation mode.");
	}

	auto SetFragment = [](Fragment pFrag, ClippedVertex const& start_, ClippedVertex const& end_, int x, int y, int fullX, std::function<void(Fragment const&)> const& emit_fragment) {
		pFrag.fb_position.x = (float)(x + 0.5);
		pFrag.fb_position.y = (float)(y + 0.5);
		pFrag.fb_position.z = end_.fb_position.z * abs(pFrag.fb_position.y - start_.fb_position.y) / abs(end_.fb_position.y - start_.fb_position.y) + start_.fb_position.z * abs(pFrag.fb_position.y - end_.fb_position.y) / abs(end_.fb_position.y - start_.fb_position.y);//interpolate z
		pFrag.attributes = start_.attributes;
		pFrag.derivatives.fill(Vec2(0.0f, 0.0f));
		emit_fragment(pFrag);
		return 0;
	};

	float x1 = va.fb_position.x;
	float x2 = vb.fb_position.x;
	float y1 = va.fb_position.y;
	float y2 = vb.fb_position.y;
	int sw = 0;

	if(x1 > x2){
		std::swap(x1,x2);
		std::swap(y1,y2);
		sw = 1;
	}

	int dx = (int)floor(x2) - (int)floor(x1);
	int dy = (int)floor(y2) - (int)floor(y1);
	int x = (int)floor(x1);
	int y = (int)floor(y1);
	int eps = 0;

	
	if(dy >= 0 && dy <= dx)
	{
		for(; x <= x2 ; x++) {
			if( sw == 0 && x==(int)x2 && ((abs(x2-(int)x2-0.5) + abs(y2-(int)y2-0.5) < 0.5) || (x2-(int)x2 == 0.5 && y2-(int)y2 == 0) || (x2-(int)x2 == 0 && y2-(int)y2 == 0.5)) ){}
			else if( sw == 1 && x==(int)x1 && ((abs(x1-(int)x1-0.5) + abs(y1-(int)y1-0.5) < 0.5) || (x1-(int)x1 == 0.5 && y1-(int)y1 == 0) || (x1-(int)x1 == 0 && y1-(int)y1 == 0.5)) ){}
			else if( x==(int)x2 && (((x2-(int)x2)+(y2-(int)y2)) < 0.5 || ((y2-(int)y2)-(x2-(int)x2))> 0.5)){}
			else if( x==(int)x1 && (((x1-(int)x1)+(y1-(int)y1)) > 1.5 || ((y1-(int)y1)-(x1-(int)x1))< -0.5)){}
			else{
				Fragment mid;
				SetFragment(mid, va, vb, x, y, dx, emit_fragment);
			}

			eps += dy;
			if((eps << 1) >= dx){
					y++;
					eps -= dx;
				}
		}
	}
	else if(dy >= 0 && dy > dx)
	{
		for(; y <= y2; y++ ){
			if( sw == 0 && y==(int)y2 && ((abs(x2-(int)x2-0.5) + abs(y2-(int)y2-0.5) < 0.5) || (x2-(int)x2 == 0.5 && y2-(int)y2 == 0) || (x2-(int)x2 == 0 && y2-(int)y2 == 0.5)) ){continue;}
			else if( sw == 1 && y==(int)y1 && ((abs(x1-(int)x1-0.5) + abs(y1-(int)y1-0.5) < 0.5) || (x1-(int)x1 == 0.5 && y1-(int)y1 == 0) || (x1-(int)x1 == 0 && y1-(int)y1 == 0.5)) ){continue;}
			else if( y==(int)y2 && ((x2-(int)x2)+(y2-(int)y2) < 0.5 || ((y2-(int)y2)-(x2-(int)x2))< -0.5)){continue;}
			else if( y==(int)y1 && (((x1-(int)x1)+(y1-(int)y1)) > 1.5 || ((y1-(int)y1)-(x1-(int)x1))> 0.5)){continue;}
			else{
				Fragment mid;
				SetFragment(mid, va, vb, x, y, dx, emit_fragment);
			}

			eps += dx;
			if((eps << 1) >= dy){
				x++;
				eps -= dy;
			}
		}
	}
	else if(dy < 0 && dy >= -dx)
	{
		for(; x <= x2; x++){
			if( sw == 0 && x==(int)x2 &&((abs(x2-(int)x2-0.5) + abs(y2-(int)y2-0.5) < 0.5) || (x2-(int)x2 == 0.5 && y2-(int)y2 == 0) || (x2-(int)x2 == 0 && y2-(int)y2 == 0.5)) ){continue;}
			else if( sw == 1 && x==(int)x1 && ((abs(x1-(int)x1-0.5) + abs(y1-(int)y1-0.5) < 0.5) || (x1-(int)x1 == 0.5 && y1-(int)y1 == 0) || (x1-(int)x1 == 0 && y1-(int)y1 == 0.5)) ){continue;}
			else if( x==(int)x2 && (((x2-(int)x2)+(y2-(int)y2)) < 0.5 || ((y2-(int)y2)-(x2-(int)x2))> 0.5)){continue;}
			else if( x==(int)x1 && (((x1-(int)x1)+(y1-(int)y1)) > 1.5 || ((y1-(int)y1)-(x1-(int)x1))< -0.5)){continue;}
			else{
				Fragment mid;
				SetFragment(mid, va, vb, x, y, dx, emit_fragment);
			}
			eps += dy;
			if((eps << 1) <= -dx){
				y--;
				eps += dx;
			}
		}
	}
	else if( dy < 0 && dy < -dx)
	{
		for(; y >= y2; y--){
			if( sw == 0 && y==(int)y2 && ((abs(x2-(int)x2-0.5) + abs(y2-(int)y2-0.5) < 0.5) || (x2-(int)x2 == 0.5 && y2-(int)y2 == 0) || (x2-(int)x2 == 0 && y2-(int)y2 == 0.5))){continue;}
			else if( sw == 1 && y==(int)y1 && ((abs(x1-(int)x1-0.5) + abs(y1-(int)y1-0.5) < 0.5) || (x1-(int)x1 == 0.5 && y1-(int)y1 == 0) || (x1-(int)x1 == 0 && y1-(int)y1 == 0.5))){continue;}
			else if( y==(int)y2 && (((x2-(int)x2)+(y2-(int)y2)) > 1.5 || ((y2-(int)y2)-(x2-(int)x2))> 0.5)){continue;}
			else if( y==(int)y1 && (((x1-(int)x1)+(y1-(int)y1)) < 0.5 || ((y1-(int)y1)-(x1-(int)x1))< -0.5)){continue;}
			else{
				Fragment mid;
				SetFragment(mid, va, vb, x, y, dx, emit_fragment);
			}
			eps -= dx;
			if((eps << 1) <= dy){
				x++;
				eps -= dy;
			}
		}
	}
}

/*
 * rasterize_triangle(a,b,c,emit) calls 'emit(frag)' at every location
 *  	(x+0.5,y+0.5) (where x,y are integers) covered by triangle (a,b,c).
 *
 * The emitted fragment should have:
 * - frag.fb_position.xy = (x+0.5, y+0.5)
 * - frag.fb_position.z = linearly interpolated fb_position.z from a,b,c (NOTE: does not depend on Interp mode!)
 * - frag.attributes = depends on Interp_* flag in flags:
 *   - if Interp_Flat: copy from va.attributes
 *   - if Interp_Smooth: interpolate as if (a,b,c) is a 2D triangle flat on the screen
 *   - if Interp_Correct: use perspective-correct interpolation
 * - frag.derivatives = derivatives w.r.t. fb_position.x and fb_position.y of the first frag.derivatives.size() attributes.
 *
 * Notes on derivatives:
 * 	The derivatives are partial derivatives w.r.t. screen locations. That is:
 *    derivatives[i].x = d/d(fb_position.x) attributes[i]
 *    derivatives[i].y = d/d(fb_position.y) attributes[i]
 *  You may compute these derivatives analytically or numerically.
 *
 *  See section 8.12.1 "Derivative Functions" of the GLSL 4.20 specification for some inspiration. (*HOWEVER*, the spec is solving a harder problem, and also nothing in the spec is binding on your implementation)
 *
 *  One approach is to rasterize blocks of four fragments and use forward and backward differences to compute derivatives.
 *  To assist you in this approach, keep in mind that the framebuffer size is *guaranteed* to be even. (see framebuffer.h)
 *
 * Notes on coverage:
 *  If two triangles are on opposite sides of the same edge, and a
 *  fragment center lies on that edge, rasterize_triangle should
 *  make sure that exactly one of the triangles emits that fragment.
 *  (Otherwise, speckles or cracks can appear in the final render.)
 * 
 *  For degenerate (co-linear) triangles, you may consider them to not be on any side of an edge.
 * 	Thus, even if two degnerate triangles share an edge that contains a fragment center, you don't need to emit it.
 *  You will not lose points for doing something reasonable when handling this case
 *
 *  This is pretty tricky to get exactly right!
 *
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_triangle(
	ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc,
	std::function<void(Fragment const&)> const& emit_fragment) {

	//cross product in 2D
	auto cross2D = [](Vec2 a, Vec2 b) -> float {
		return a.x * b.y - a.y * b.x;
	};

	//calculate the unsigned area of triangle by cross product
	auto TriArea = [&cross2D] (Vec2 x, Vec2 y){
		return abs(cross2D(x,y)) / 2.0f;
	};

	//calculate the signed area of triangle by cross product
	auto SignedTriArea = [&cross2D] (Vec2 x, Vec2 y){
		return cross2D(x,y) / 2.0f;
	};

	//test if the va-vb is top or left edge
	auto TestTopOrLeft = [&cross2D](ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc)->bool {

		//judge top edges:
		if(va.fb_position.y == vb.fb_position.y && va.fb_position.y > vc.fb_position.y){
			return true;
		}
			
		//judge left edges:
		float dy = vb.fb_position.y - va.fb_position.y;
		auto abVec = Vec2(vb.fb_position.x - va.fb_position.x, vb.fb_position.y - va.fb_position.y);
		auto acVec = Vec2(vc.fb_position.x - va.fb_position.x, vc.fb_position.y - va.fb_position.y);

		if(dy * cross2D(abVec, acVec) < 0){
			return true;
		}
		
		return false;
	};

	int left = (int)std::min(va.fb_position.x, std::min(vb.fb_position.x, vc.fb_position.x));
	int top = (int)std::max(va.fb_position.y, std::max(vb.fb_position.y, vc.fb_position.y));
	int right = (int)std::max(va.fb_position.x, std::max(vb.fb_position.x, vc.fb_position.x));
	int bottom = (int)std::min(va.fb_position.y, std::min(vb.fb_position.y, vc.fb_position.y));

	//initialize vector ab & ac
	auto abVec = Vec2(vb.fb_position.x - va.fb_position.x, vb.fb_position.y - va.fb_position.y);
	auto acVec = Vec2(vc.fb_position.x - va.fb_position.x, vc.fb_position.y - va.fb_position.y);

	//initialize vector ba & bc
	auto baVec = Vec2(va.fb_position.x - vb.fb_position.x, va.fb_position.y - vb.fb_position.y);
	auto bcVec = Vec2(vc.fb_position.x - vb.fb_position.x, vc.fb_position.y - vb.fb_position.y);

	//initialize vector ca & cb
	auto caVec = Vec2(va.fb_position.x - vc.fb_position.x, va.fb_position.y - vc.fb_position.y);
	auto cbVec = Vec2(vb.fb_position.x - vc.fb_position.x, vb.fb_position.y - vc.fb_position.y);

	//calculate the area of triangle
	float TriFullS = TriArea(abVec, acVec);

	if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
		// A1T3: flat triangles

		//judge if it's a triangle
		if(TriFullS == 0)
		{
			return;
		}

		//set fragment function - flat
		auto SetFragment = [&TriArea](Fragment Pixel, Vec2 P_center, Vec2 aPVec, Vec2 bPVec, Vec2 cPVec, Vec2 acVec, Vec2 cbVec, Vec2 baVec, ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc, float TriFullS, std::function<void(Fragment const&)> const& emit_fragment) {
			Pixel.fb_position.x = P_center.x;
			Pixel.fb_position.y = P_center.y;
			Pixel.fb_position.z = TriArea(acVec, aPVec)*vb.fb_position.z/TriFullS + TriArea(cbVec, cPVec)*va.fb_position.z/TriFullS + TriArea(baVec, bPVec)*vc.fb_position.z/TriFullS;
			Pixel.attributes = va.attributes;
			Pixel.derivatives.fill(Vec2(0.0f, 0.0f));
			emit_fragment(Pixel);
			return 0;
		};	

		//rasterize triangle
		for( int x = left; x <= right; x++){
			for( int y = bottom; y <= top; y++){
				auto P_center = Vec2((float)(x+0.5), (float)(y+0.5));
				auto aPVec = Vec2(P_center.x - va.fb_position.x, P_center.y - va.fb_position.y);
				auto bPVec = Vec2(P_center.x - vb.fb_position.x, P_center.y - vb.fb_position.y);
				auto cPVec = Vec2(P_center.x - vc.fb_position.x, P_center.y - vc.fb_position.y);

				float judge1 = cross2D(acVec, abVec) * cross2D(acVec, aPVec);
				float judge2 = cross2D(cbVec, caVec) * cross2D(cbVec, cPVec);
				float judge3 = cross2D(baVec, bcVec) * cross2D(baVec, bPVec);

				if( judge1 > 0 && judge2 > 0 && judge3 > 0){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 == 0 && judge2 > 0 && judge3 > 0 && TestTopOrLeft(va, vc, vb)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 > 0 && judge2 == 0 && judge3 > 0 && TestTopOrLeft(vb, vc, va)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 > 0 && judge2 > 0 && judge3 == 0 && TestTopOrLeft(vb, va, vc)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 == 0 && judge2 == 0 && judge3 > 0 && TestTopOrLeft(va, vc, vb) && TestTopOrLeft(vb, vc, va)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 == 0 && judge2 > 0 && judge3 == 0 && TestTopOrLeft(va, vc, vb) && TestTopOrLeft(vb, va, vc)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 > 0 && judge2 == 0 && judge3 == 0 && TestTopOrLeft(vb, vc, va) && TestTopOrLeft(vb, va, vc)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, va, vb, vc, TriFullS, emit_fragment);
				}
			}
		}

	} else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Smooth) {
		// A1T5: screen-space smooth triangles
		// TODO: rasterize triangle (see block comment above this function).

		//judge if it's a triangle
		if(TriFullS == 0)
		{
			return;
		}

		bool CCW = false;
		if(cross2D(acVec, abVec) > 0){
			CCW = true;
		}

		//set fragment function - smooth
		auto SetFragment = [&TriArea](Fragment Pixel, Vec2 P_center, Vec2 aPVec, Vec2 bPVec, Vec2 cPVec, Vec2 acVec, Vec2 cbVec, Vec2 baVec, Vec2 abVec, Vec2 bcVec, Vec2 caVec, bool CCW, ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc, float TriFullS, std::function<void(Fragment const&)> const& emit_fragment) {
			Pixel.fb_position.x = P_center.x;
			Pixel.fb_position.y = P_center.y;
			Pixel.fb_position.z = TriArea(acVec, aPVec)*vb.fb_position.z/TriFullS + TriArea(cbVec, cPVec)*va.fb_position.z/TriFullS + TriArea(baVec, bPVec)*vc.fb_position.z/TriFullS;

			for(uint32_t i = 0; i < va.attributes.size(); i++){
				Pixel.attributes[i] = TriArea(acVec, aPVec)*vb.attributes[i]/TriFullS + TriArea(cbVec, cPVec)*va.attributes[i]/TriFullS + TriArea(baVec, bPVec)*vc.attributes[i]/TriFullS;
			}

			for(uint32_t i = 0; i < Pixel.derivatives.size(); i++){
				if(CCW){
					Pixel.derivatives[i].x = (caVec.y * vb.attributes[i] + bcVec.y * va.attributes[i] + abVec.y * vc.attributes[i]) / 2 * TriFullS;
					Pixel.derivatives[i].y = (acVec.x * vb.attributes[i] + cbVec.x * va.attributes[i] + baVec.x * vc.attributes[i]) / 2 * TriFullS;
				}
				else{
					Pixel.derivatives[i].x = (acVec.y * vb.attributes[i] + cbVec.y * va.attributes[i] + baVec.y * vc.attributes[i]) / 2 * TriFullS;
					Pixel.derivatives[i].y = (caVec.x * vb.attributes[i] + bcVec.x * va.attributes[i] + abVec.x * vc.attributes[i]) / 2 * TriFullS;
				}
			}
			emit_fragment(Pixel);
			return 0;
		};

		//rasterize triangle
		for( int x = left; x <= right; x++){
			for( int y = bottom; y <= top; y++){
				auto P_center = Vec2((float)(x+0.5), (float)(y+0.5));
				auto aPVec = Vec2(P_center.x - va.fb_position.x, P_center.y - va.fb_position.y);
				auto bPVec = Vec2(P_center.x - vb.fb_position.x, P_center.y - vb.fb_position.y);
				auto cPVec = Vec2(P_center.x - vc.fb_position.x, P_center.y - vc.fb_position.y);

				float judge1 = cross2D(acVec, abVec) * cross2D(acVec, aPVec);
				float judge2 = cross2D(cbVec, caVec) * cross2D(cbVec, cPVec);
				float judge3 = cross2D(baVec, bcVec) * cross2D(baVec, bPVec);

				if( judge1 > 0 && judge2 > 0 && judge3 > 0){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, abVec, bcVec, caVec, CCW, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 == 0 && judge2 > 0 && judge3 > 0 && TestTopOrLeft(va, vc, vb)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, abVec, bcVec, caVec, CCW, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 > 0 && judge2 == 0 && judge3 > 0 && TestTopOrLeft(vb, vc, va)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, abVec, bcVec, caVec, CCW, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 > 0 && judge2 > 0 && judge3 == 0 && TestTopOrLeft(vb, va, vc)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, abVec, bcVec, caVec, CCW, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 == 0 && judge2 == 0 && judge3 > 0 && TestTopOrLeft(va, vc, vb) && TestTopOrLeft(vb, vc, va)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, abVec, bcVec, caVec, CCW, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 == 0 && judge2 > 0 && judge3 == 0 && TestTopOrLeft(va, vc, vb) && TestTopOrLeft(vb, va, vc)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, abVec, bcVec, caVec, CCW, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 > 0 && judge2 == 0 && judge3 == 0 && TestTopOrLeft(vb, vc, va) && TestTopOrLeft(vb, va, vc)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, abVec, bcVec, caVec, CCW, va, vb, vc, TriFullS, emit_fragment);
				}
			}
		}

	} else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Correct) {
		// A1T5: perspective correct triangles
		// TODO: rasterize triangle (block comment above this function).

		//judge if it's a triangle
		if(TriFullS == 0)
		{
			return;
		}

		bool CCW = false;
		if(cross2D(acVec, abVec) > 0){
			CCW = true;
		}

		//set fragment function - correct
		auto SetFragment = [&TriArea, &SignedTriArea](Fragment Pixel, Vec2 P_center, Vec2 aPVec, Vec2 bPVec, Vec2 cPVec, Vec2 acVec, Vec2 cbVec, Vec2 baVec, Vec2 abVec, Vec2 bcVec, Vec2 caVec, bool CCW, ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc, float TriFullS, std::function<void(Fragment const&)> const& emit_fragment) {
			Pixel.fb_position.x = P_center.x;
			Pixel.fb_position.y = P_center.y;
			Pixel.fb_position.z = TriArea(acVec, aPVec)*vb.fb_position.z/TriFullS + TriArea(cbVec, cPVec)*va.fb_position.z/TriFullS + TriArea(baVec, bPVec)*vc.fb_position.z/TriFullS;
			Pixel.derivatives.fill(Vec2(0.0f, 0.0f));
			//compute the 1/w value at the (x, y) pixel
			float Pixel_w = TriArea(acVec, aPVec)*vb.inv_w/TriFullS + TriArea(cbVec, cPVec)*va.inv_w/TriFullS + TriArea(baVec, bPVec)*vc.inv_w/TriFullS;
			
			//initialize x+1 Fragment
			Fragment PixelX1;
			PixelX1.fb_position.x = P_center.x + 1;
			PixelX1.fb_position.y = P_center.y;
			auto aPX1Vec = Vec2(PixelX1.fb_position.x - va.fb_position.x, PixelX1.fb_position.y - va.fb_position.y);
			auto bPX1Vec = Vec2(PixelX1.fb_position.x - vb.fb_position.x, PixelX1.fb_position.y - vb.fb_position.y);
			auto cPX1Vec = Vec2(PixelX1.fb_position.x - vc.fb_position.x, PixelX1.fb_position.y - vc.fb_position.y);
			if(CCW){
				//compute the 1/w value at the (x+1, y) pixel
				float PixelX1_w = SignedTriArea(acVec, aPX1Vec)*vb.inv_w/TriFullS + SignedTriArea(cbVec, cPX1Vec)*va.inv_w/TriFullS + SignedTriArea(baVec, bPX1Vec)*vc.inv_w/TriFullS;
				for(uint32_t i = 0; i < va.attributes.size(); i++){
					PixelX1.attributes[i] = (SignedTriArea(acVec, aPX1Vec)*vb.attributes[i]*vb.inv_w/TriFullS + SignedTriArea(cbVec, cPX1Vec)*va.attributes[i]*va.inv_w/TriFullS + SignedTriArea(baVec, bPX1Vec)*vc.attributes[i]*vc.inv_w/TriFullS)/PixelX1_w;
				}
			}
			else{
				//compute the 1/w value at the (x, y+1) pixel
				float PixelX1_w = SignedTriArea(abVec, aPX1Vec)*vc.inv_w/TriFullS + SignedTriArea(bcVec, bPX1Vec)*va.inv_w/TriFullS + SignedTriArea(caVec, cPX1Vec)*vb.inv_w/TriFullS;
				for(uint32_t i = 0; i < va.attributes.size(); i++){
					PixelX1.attributes[i] = (SignedTriArea(abVec, aPX1Vec)*vc.attributes[i]*vc.inv_w + SignedTriArea(bcVec, bPX1Vec)*va.attributes[i]*va.inv_w/TriFullS + SignedTriArea(caVec, cPX1Vec)*vb.attributes[i]*vb.inv_w/TriFullS)/PixelX1_w;
				}
			}

			//initialize x-1 Fragment
			Fragment PixelX_1;
			PixelX_1.fb_position.x = P_center.x - 1;
			PixelX_1.fb_position.y = P_center.y;
			auto aPX_1Vec = Vec2(PixelX_1.fb_position.x - va.fb_position.x, PixelX_1.fb_position.y - va.fb_position.y);
			auto bPX_1Vec = Vec2(PixelX_1.fb_position.x - vb.fb_position.x, PixelX_1.fb_position.y - vb.fb_position.y);
			auto cPX_1Vec = Vec2(PixelX_1.fb_position.x - vc.fb_position.x, PixelX_1.fb_position.y - vc.fb_position.y);
			if(CCW){
				//compute the 1/w value at the (x+1, y) pixel
				float PixelX_w = SignedTriArea(acVec, aPX_1Vec)*vb.inv_w/TriFullS + SignedTriArea(cbVec, cPX_1Vec)*va.inv_w/TriFullS + SignedTriArea(baVec, bPX_1Vec)*vc.inv_w/TriFullS;
				for(uint32_t i = 0; i < va.attributes.size(); i++){
					PixelX_1.attributes[i] = (SignedTriArea(acVec, aPX_1Vec)*vb.attributes[i]*vb.inv_w/TriFullS + SignedTriArea(cbVec, cPX_1Vec)*va.attributes[i]*va.inv_w/TriFullS + SignedTriArea(baVec, bPX_1Vec)*vc.attributes[i]*vc.inv_w/TriFullS)/PixelX_w;
				}
			}
			else{
				//compute the 1/w value at the (x, y+1) pixel
				float PixelX_w = SignedTriArea(abVec, aPX_1Vec)*vc.inv_w/TriFullS + SignedTriArea(bcVec, bPX_1Vec)*va.inv_w/TriFullS + SignedTriArea(caVec, cPX_1Vec)*vb.inv_w/TriFullS;
				for(uint32_t i = 0; i < va.attributes.size(); i++){
					PixelX_1.attributes[i] = (SignedTriArea(abVec, aPX_1Vec)*vc.attributes[i]*vc.inv_w + SignedTriArea(bcVec, bPX_1Vec)*va.attributes[i]*va.inv_w/TriFullS + SignedTriArea(caVec, cPX_1Vec)*vb.attributes[i]*vb.inv_w/TriFullS)/PixelX_w;
				}
			}

			//initialize y+1 Fragment
			Fragment PixelY1;
			PixelY1.fb_position.x = P_center.x;
			PixelY1.fb_position.y = P_center.y + 1;
			auto aPY1Vec = Vec2(PixelY1.fb_position.x - va.fb_position.x, PixelY1.fb_position.y - va.fb_position.y);
			auto bPY1Vec = Vec2(PixelY1.fb_position.x - vb.fb_position.x, PixelY1.fb_position.y - vb.fb_position.y);
			auto cPY1Vec = Vec2(PixelY1.fb_position.x - vc.fb_position.x, PixelY1.fb_position.y - vc.fb_position.y);
			if(CCW){
				float PixelY_w = SignedTriArea(acVec, aPY1Vec)*vb.inv_w/TriFullS + SignedTriArea(cbVec, cPY1Vec)*va.inv_w/TriFullS + SignedTriArea(baVec, bPY1Vec)*vc.inv_w/TriFullS;
				for(uint32_t i = 0; i < va.attributes.size(); i++){
					PixelY1.attributes[i] = (SignedTriArea(acVec, aPY1Vec)*vb.attributes[i]*vb.inv_w/TriFullS + SignedTriArea(cbVec, cPY1Vec)*va.attributes[i]*va.inv_w/TriFullS + SignedTriArea(baVec, bPY1Vec)*vc.attributes[i]*vc.inv_w/TriFullS)/PixelY_w;
				}
			}
			else{
				float PixelY_w = SignedTriArea(abVec, aPY1Vec)*vc.inv_w/TriFullS + SignedTriArea(bcVec, bPY1Vec)*va.inv_w/TriFullS + SignedTriArea(caVec, cPY1Vec)*vb.inv_w/TriFullS;
				for(uint32_t i = 0; i < va.attributes.size(); i++){
					PixelY1.attributes[i] = (SignedTriArea(abVec, aPY1Vec)*vc.attributes[i]*vc.inv_w/TriFullS + SignedTriArea(bcVec, bPY1Vec)*va.attributes[i]*va.inv_w/TriFullS + SignedTriArea(caVec, cPY1Vec)*vb.attributes[i]*vb.inv_w/TriFullS)/PixelY_w;
				}
			}

			//initialize y-1 Fragment
			Fragment PixelY_1;
			PixelY_1.fb_position.x = P_center.x;
			PixelY_1.fb_position.y = P_center.y - 1;
			auto aPY_1Vec = Vec2(PixelY_1.fb_position.x - va.fb_position.x, PixelY_1.fb_position.y - va.fb_position.y);
			auto bPY_1Vec = Vec2(PixelY_1.fb_position.x - vb.fb_position.x, PixelY_1.fb_position.y - vb.fb_position.y);
			auto cPY_1Vec = Vec2(PixelY_1.fb_position.x - vc.fb_position.x, PixelY_1.fb_position.y - vc.fb_position.y);
			if(CCW){
				float PixelY_w = SignedTriArea(acVec, aPY_1Vec)*vb.inv_w/TriFullS + SignedTriArea(cbVec, cPY_1Vec)*va.inv_w/TriFullS + SignedTriArea(baVec, bPY_1Vec)*vc.inv_w/TriFullS;
				for(uint32_t i = 0; i < va.attributes.size(); i++){
					PixelY_1.attributes[i] = (SignedTriArea(acVec, aPY_1Vec)*vb.attributes[i]*vb.inv_w/TriFullS + SignedTriArea(cbVec, cPY_1Vec)*va.attributes[i]*va.inv_w/TriFullS + SignedTriArea(baVec, bPY_1Vec)*vc.attributes[i]*vc.inv_w/TriFullS)/PixelY_w;
				}
			}
			else{
				float PixelY_w = SignedTriArea(abVec, aPY_1Vec)*vc.inv_w/TriFullS + SignedTriArea(bcVec, bPY_1Vec)*va.inv_w/TriFullS + SignedTriArea(caVec, cPY_1Vec)*vb.inv_w/TriFullS;
				for(uint32_t i = 0; i < va.attributes.size(); i++){
					PixelY_1.attributes[i] = (SignedTriArea(abVec, aPY_1Vec)*vc.attributes[i]*vc.inv_w/TriFullS + SignedTriArea(bcVec, bPY_1Vec)*va.attributes[i]*va.inv_w/TriFullS + SignedTriArea(caVec, cPY_1Vec)*vb.attributes[i]*vb.inv_w/TriFullS)/PixelY_w;
				}
			}

			//interpolate Fragment
			for(uint32_t i = 0; i < va.attributes.size(); i++){
				Pixel.attributes[i] = (TriArea(acVec, aPVec)*vb.attributes[i]*vb.inv_w/TriFullS + TriArea(cbVec, cPVec)*va.attributes[i]*va.inv_w/TriFullS + TriArea(baVec, bPVec)*vc.attributes[i]*vc.inv_w/TriFullS)/Pixel_w;
			}

			for(uint32_t i = 0; i < Pixel.derivatives.size(); i++){
				Pixel.derivatives[i] = Vec2((PixelX1.attributes[i] - PixelX_1.attributes[i])/2, (PixelY1.attributes[i] - PixelY_1.attributes[i])/2);
			}

			emit_fragment(Pixel);
			return 0;
		};

		//rasterize triangle
		for( int x = left; x <= right; x++){
			for( int y = bottom; y <= top; y++){
				auto P_center = Vec2((float)(x+0.5), (float)(y+0.5));
				auto aPVec = Vec2(P_center.x - va.fb_position.x, P_center.y - va.fb_position.y);
				auto bPVec = Vec2(P_center.x - vb.fb_position.x, P_center.y - vb.fb_position.y);
				auto cPVec = Vec2(P_center.x - vc.fb_position.x, P_center.y - vc.fb_position.y);

				float judge1 = cross2D(acVec, abVec) * cross2D(acVec, aPVec);
				float judge2 = cross2D(cbVec, caVec) * cross2D(cbVec, cPVec);
				float judge3 = cross2D(baVec, bcVec) * cross2D(baVec, bPVec);

				if( judge1 > 0 && judge2 > 0 && judge3 > 0){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, abVec, bcVec, caVec, CCW, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 == 0 && judge2 > 0 && judge3 > 0 && TestTopOrLeft(va, vc, vb)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, abVec, bcVec, caVec, CCW, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 > 0 && judge2 == 0 && judge3 > 0 && TestTopOrLeft(vb, vc, va)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, abVec, bcVec, caVec, CCW, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 > 0 && judge2 > 0 && judge3 == 0 && TestTopOrLeft(vb, va, vc)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, abVec, bcVec, caVec, CCW, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 == 0 && judge2 == 0 && judge3 > 0 && TestTopOrLeft(va, vc, vb) && TestTopOrLeft(vb, vc, va)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, abVec, bcVec, caVec, CCW, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 == 0 && judge2 > 0 && judge3 == 0 && TestTopOrLeft(va, vc, vb) && TestTopOrLeft(vb, va, vc)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, abVec, bcVec, caVec, CCW, va, vb, vc, TriFullS, emit_fragment);
				}
				else if( judge1 > 0 && judge2 == 0 && judge3 == 0 && TestTopOrLeft(vb, vc, va) && TestTopOrLeft(vb, va, vc)){
					Fragment Pixel;
					SetFragment(Pixel, P_center, aPVec, bPVec, cPVec, acVec, cbVec, baVec, abVec, bcVec, caVec, CCW, va, vb, vc, TriFullS, emit_fragment);
				}
			}
		}
	}
}

//-------------------------------------------------------------------------
// compile instantiations for all programs and blending and testing types:

#include "programs.h"

template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Flat>;