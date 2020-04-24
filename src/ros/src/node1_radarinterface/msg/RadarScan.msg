# header info
Header header

# original measurement from radar
# PointCloudData (DPIF_PointCloudSpherical_t in mmwave_sdk)
float32 range
float32 angle
float32 elev
float32 doppler
# SideInfoData (DPIF_PointCloudSideInfo_t in mmwave_sdk)
# CFAR cell to side noise ratio in dB expressed in 0.1 steps of dB
uint16 snr
# CFAR noise level of the side of the detected cell in dB expressed in 0.1 steps of dB
uint16 noise

# Point in the Cartesian coordinates
uint32 frame_num
uint16 point_id
float32 x
float32 y
float32 z
uint16 doppler_bin

# Target centriod in the Cartesian coordinates
# The meaning of the target index greater than 252 in Gtrack
# define GTRACK_ID_POINT_TOO_WEAK            (253U)   /**< @brief Point is not associated, is too weak */
# define GTRACK_ID_POINT_BEHIND_THE_WALL     (254U)   /**< @brief Point is not associated, is behind the wall */
# define GTRACK_ID_POINT_NOT_ASSOCIATED      (255U)   /**< @brief Point is not associated, noise */
uint16 target_idx
float32 posX
float32 posY
float32 posZ
float32 velX
float32 velY
float32 velZ
float32 accX
float32 accY
float32 accZ
