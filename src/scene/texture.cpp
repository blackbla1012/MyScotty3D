
#include "texture.h"

#include <iostream>

namespace Textures {


Spectrum sample_nearest(HDR_Image const &image, Vec2 uv) {
	//clamp texture coordinates, convert to [0,w]x[0,h] pixel space:
	float x = image.w * std::clamp(uv.x, 0.0f, 1.0f);
	float y = image.h * std::clamp(uv.y, 0.0f, 1.0f);

	//the pixel with the nearest center is the pixel that contains (x,y):
	int32_t ix = int32_t(std::floor(x));
	int32_t iy = int32_t(std::floor(y));

	//texture coordinates of (1,1) map to (w,h), and need to be reduced:
	ix = std::min(ix, int32_t(image.w) - 1);
	iy = std::min(iy, int32_t(image.h) - 1);

	return image.at(ix, iy);
}

Spectrum sample_bilinear(HDR_Image const &image, Vec2 uv) {
	// A1T6: sample_bilinear
	//TODO: implement bilinear sampling strategy on texture 'image'
	float x = image.w * std::clamp(uv.x, 0.0f, 1.0f);
	float y = image.h * std::clamp(uv.y, 0.0f, 1.0f);

	int32_t ix = int32_t(std::round(x)) - 1;
	int32_t iy = int32_t(std::round(y)) - 1;

	ix = std::clamp(ix, 0, int32_t(image.w) - 1);
	iy = std::clamp(iy, 0, int32_t(image.h) - 1);

	float dx = x - ix - 0.5f;
	float dy = y - iy - 0.5f;

	Spectrum Texel_XY = image.at(ix, iy);
	Spectrum Texel_X1Y = (ix + 1 < (int32_t)image.w ? image.at(ix + 1, iy) : Texel_XY);
	Spectrum Texel_XY1 = (iy + 1 < (int32_t)image.h ? image.at(ix, iy + 1) : Texel_XY);
	Spectrum Texel_X1Y1 = (ix + 1 < (int32_t)image.w && iy + 1 < (int32_t)image.h ? image.at(ix + 1, iy + 1) : Texel_XY1);

	Spectrum Texel_X = (1 - dx) * Texel_XY + dx * Texel_X1Y;
	Spectrum Texel_Y = (1 - dx) * Texel_XY1 + dx * Texel_X1Y1;

	Spectrum Texel = (1 - dy) * Texel_X + dy * Texel_Y;

	return Texel; 
}


Spectrum sample_trilinear(HDR_Image const &base, std::vector< HDR_Image > const &levels, Vec2 uv, float lod) {
	// A1T6: sample_trilinear
	//TODO: implement trilinear sampling strategy on using mip-map 'levels'
	int32_t lod0 = int32_t(std::floor(lod));

	float lodMax = (float)std::log2(std::max(base.w, base.h));
	lod0 = std::clamp(lod0, 0, int32_t(std::floor(lodMax)));

	float dlod = lod - lod0;

	Spectrum Texel_d = sample_bilinear(base, uv);
	Spectrum Texel_d1 = sample_bilinear(base, uv);

	if(lod0 < int32_t(std::floor(lodMax))){
		Texel_d = (lod0 == 0 ? sample_bilinear(base, uv) : sample_bilinear(levels[lod0-1], uv));
		Texel_d1 = sample_bilinear(levels[lod0], uv);
	}
	else if(lod0 == 0 && lod0 == int32_t(std::floor(lodMax))){
		Texel_d = sample_bilinear(base, uv);
		Texel_d1 = Texel_d;
	}
	else if(lod0 == int32_t(std::floor(lodMax))){
		Texel_d = sample_bilinear(levels[lod0-1], uv);
		Texel_d1 = Texel_d;
	}

	Spectrum Texel = (1 - dlod) * Texel_d + dlod * Texel_d1;

	return Texel;
}

/*
 * generate_mipmap- generate mipmap levels from a base image.
 *  base: the base image
 *  levels: pointer to vector of levels to fill (must not be null)
 *
 * generates a stack of levels [1,n] of sizes w_i, h_i, where:
 *   w_i = max(1, floor(w_{i-1})/2)
 *   h_i = max(1, floor(h_{i-1})/2)
 *  with:
 *   w_0 = base.w
 *   h_0 = base.h
 *  and n is the smalles n such that w_n = h_n = 1
 *
 * each level should be calculated by downsampling a blurred version
 * of the previous level to remove high-frequency detail.
 *
 */
void generate_mipmap(HDR_Image const &base, std::vector< HDR_Image > *levels_) {
	assert(levels_);
	auto &levels = *levels_;


	{ // allocate sublevels sufficient to scale base image all the way to 1x1:
		int32_t num_levels = static_cast<int32_t>(std::log2(std::max(base.w, base.h)));
		assert(num_levels >= 0);

		levels.clear();
		levels.reserve(num_levels);

		uint32_t width = base.w;
		uint32_t height = base.h;
		for (int32_t i = 0; i < num_levels; ++i) {
			assert(!(width == 1 && height == 1)); //would have stopped before this if num_levels was computed correctly

			width = std::max(1u, width / 2u);
			height = std::max(1u, height / 2u);

			levels.emplace_back(width, height);
		}
		assert(width == 1 && height == 1);
		assert(levels.size() == uint32_t(num_levels));
	}

	//now fill in the levels using a helper:
	//downsample:
	// fill in dst to represent the low-frequency component of src
	auto downsample = [](HDR_Image const &src, HDR_Image &dst) {
		//dst is half the size of src in each dimension:
		assert(std::max(1u, src.w / 2u) == dst.w);
		assert(std::max(1u, src.h / 2u) == dst.h);

		// A1T6: generate
		//TODO: Write code to fill the levels of the mipmap hierarchy by downsampling
		assert(!(src.w == 1 && src.h == 1));
		for(uint32_t j = 0; j < dst.h; j++){
			for(uint32_t i = 0; i < dst.w; i++){
				Spectrum Texel(0.0f, 0.0f, 0.0f);
				if(src.w == 1 && src.h > 1){
					Texel = (src.at(2*i,2*j) + src.at(2*i,2*j+1))/2.0f;
				}
				else if(src.w > 1 && src.h == 1){
					Texel = (src.at(2*i,2*j) + src.at(2*i+1,2*j))/2.0f;
				}
				else if(src.w > 1 && src.h > 1){
					Texel = (src.at(2*i,2*j) + src.at(2*i+1,2*j) + src.at(2*i,2*j+1) + src.at(2*i+1,2*j+1))/4.0f;
				}
				dst.at(i,j) = Texel;
			}
		}
		//Be aware that the alignment of the samples in dst and src will be different depending on whether the image is even or odd.

	};

	std::cout << "Regenerating mipmap (" << levels.size() << " levels): [" << base.w << "x" << base.h << "]";
	std::cout.flush();
	for (uint32_t i = 0; i < levels.size(); ++i) {
		HDR_Image const &src = (i == 0 ? base : levels[i-1]);
		HDR_Image &dst = levels[i];
		std::cout << " -> [" << dst.w << "x" << dst.h << "]"; std::cout.flush();

		downsample(src, dst);
	}
	std::cout << std::endl;
	
}

Image::Image(Sampler sampler_, HDR_Image const &image_) {
	sampler = sampler_;
	image = image_.copy();
	update_mipmap();
}

Spectrum Image::evaluate(Vec2 uv, float lod) const {
	if (sampler == Sampler::nearest) {
		return sample_nearest(image, uv);
	} else if (sampler == Sampler::bilinear) {
		return sample_bilinear(image, uv);
	} else {
		return sample_trilinear(image, levels, uv, lod);
	}
}

void Image::update_mipmap() {
	if (sampler == Sampler::trilinear) {
		generate_mipmap(image, &levels);
	} else {
		levels.clear();
	}
}

GL::Tex2D Image::to_gl() const {
	return image.to_gl(1.0f);
}

void Image::make_valid() {
	update_mipmap();
}

Spectrum Constant::evaluate(Vec2 uv, float lod) const {
	return color * scale;
}

} // namespace Textures
bool operator!=(const Textures::Constant& a, const Textures::Constant& b) {
	return a.color != b.color || a.scale != b.scale;
}

bool operator!=(const Textures::Image& a, const Textures::Image& b) {
	return a.image != b.image;
}

bool operator!=(const Texture& a, const Texture& b) {
	if (a.texture.index() != b.texture.index()) return false;
	return std::visit(
		[&](const auto& data) { return data != std::get<std::decay_t<decltype(data)>>(b.texture); },
		a.texture);
}
