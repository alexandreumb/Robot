int64 timestamp
int64 publish_timestamp
uint8 is_bigendian
uint32 frequency
uint32 step_color
uint8[2764800] data_color  # 1280 * 720 * 3
uint32 step_depth
uint8[1843200] data_depth  # 1280 * 720 * 2
ImageIntrinsics image_intrinsics
