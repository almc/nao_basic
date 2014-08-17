#ifndef MOTIONS_COMMON_H_
#define MOTIONS_COMMON_H_

#include <alvalue/alvalue.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alrobotpostureproxy.h>

void CloseHand(AL::ALMotionProxy* motion_proxy_ptr);
void OpenHand(AL::ALMotionProxy* motion_proxy_ptr);

float MoveHand(AL::ALMotionProxy* motion_proxy_ptr,
               float x_hand, float y_hand, float z_hand);

void MoveHand2(AL::ALMotionProxy* motion_proxy_ptr);

void TrackBallWithHead(AL::ALMotionProxy* motion_proxy_ptr,
						int u_img, int v_img,
						int img_width, int img_height);

void CheckHand(AL::ALMotionProxy* motion_proxy_ptr);

void Bend(AL::ALMotionProxy* motion_proxy_ptr);

void Bend2(AL::ALMotionProxy* motion_proxy_ptr);

void UnBend2(AL::ALMotionProxy* motion_proxy_ptr);

void Throw(AL::ALMotionProxy* motion_proxy_ptr);

void Entangle(AL::ALMotionProxy* motion_proxy_ptr, bool hand);

bool CheckLHand(AL::ALMotionProxy* motion_proxy_ptr);

void MoveBack(AL::ALMotionProxy* motion_proxy_ptr);



#endif
