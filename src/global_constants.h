/** GlobalConstants.h */

#pragma once
#ifndef SRC_GLOBAL_CONSTANTS_H_
#define SRC_GLOBAL_CONSTANTS_H_

namespace CameraConstants {
	/**
	* maximum range represented by data structure
	*/
	const short K = 120;

	/**
	* camera focal length
	*/
	const float F = 0.035;

	/**
	* camera sensor size (meters)
	*/
	const float S = 0.0254;

	/**
	* camera x-resolution
	*/
	const float XRES = 16;

	/**
	* camera y-resolution
	*/
	const float YRES = 108;
};  // namespace CameraConstants

#endif  // SRC_GLOBAL_CONSTANTS_H_

