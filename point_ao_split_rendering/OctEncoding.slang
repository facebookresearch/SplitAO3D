// The MIT License
// Copyright © 2017 Inigo Quilez
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// https://www.shadertoy.com/view/llfcRl

// Encoding surface normals, which are 96 bits normally (pun!), into 16, 24 and 32 bits
// (6x, 4x and 3x compression respectively). Similar by Aras' blog post
//
// http://aras-p.info/texts/CompactNormalStorage.html
//
// but more complete hopefully.
//
// Left to Right: Direct, ZIgnore, Cube Spherical, Spheremap and Octahedral encoding. 
// Top is 16 bits, middle is 24 bits, bottom is 32 bits. See at full screen for best comparison.
//
// Error Color Encoding:   blue = 0.0, red = 0.5 degrees
// 
//
// Octaheral compression with different bitrates: https://www.shadertoy.com/view/Mtfyzl
//
// You can compare ithese to Fibonacci projection here: https://www.shadertoy.com/view/4t2XWK

#include "Utils/HostDeviceShared.slangh"
// This file is designed to be included on the CPU/HOST side and imported on the GPU/DEVICE side
BEGIN_NAMESPACE_FALCOR

// These defines make up for the fact that the abs functions are not overridden for the Falcor float swizzles
// and that Falcor doesn't provide a max().
#ifdef HOST_CODE
  #define abs2DSwizzle(arg) abs(Falcor::float2(arg))
  #define max2D glm::max
#else
  #define abs2DSwizzle abs
  #define max2D max
#endif

inline float2 msign( float2 v )
{
    return float2( (v.x>=0.0f) ? 1.0f : -1.0f, 
                 (v.y>=0.0f) ? 1.0f : -1.0f );
}

inline float2 unpackSnorm2x4( uint d) { return float2(uint2(d,d>> 4)& 15u)/ 7.5f - 1.0f; }

inline uint packSnorm2x4( float2 v) { uint2 d = uint2(round( 7.5f + v * 7.5f)); return d.x|(d.y<< 4u); }

inline uint octahedral8(float3 normal)
{  
  normal /= (abs(normal.x) + abs(normal.y) + abs(normal.z));
  normal.xy = (normal.z >= 0.0f) ? normal.xy : (1.0f - abs2DSwizzle(normal.yx)) * msign(normal.xy);
  return packSnorm2x4(normal.xy);
}

inline float3 octahedral8Inverse(uint data)
{
  float2 v = unpackSnorm2x4(data);
    
  float3 normal = float3(v, 1.0f - abs(v.x) - abs(v.y)); // Rune Stubbe's version,
  float t = max2D(-normal.z, 0.0f); // much faster than original
  normal.x += (normal.x > 0.0f) ? -t : t; // implementation of this
  normal.y += (normal.y > 0.0f) ? -t : t; // technique
    
  return normalize(normal);
}

END_NAMESPACE_FALCOR

//#endif