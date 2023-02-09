use crate::{_IOC, _IOC_TYPECHECK, _IOR, _IOW, _IOWR};
use crate::v4l_sys::*;
use crate::v4l2::vidioc::{_IOC_TYPE };

pub const VIDIOC_SUBDEV_QUERYCAP: _IOC_TYPE = _IOR!(b'V', 0, v4l2_subdev_capability);
pub const VIDIOC_SUBDEV_G_FMT: _IOC_TYPE = _IOWR!(b'V', 4, v4l2_subdev_format);
pub const VIDIOC_SUBDEV_S_FMT: _IOC_TYPE = _IOWR!(b'V', 5, v4l2_subdev_format);
pub const VIDIOC_SUBDEV_G_FRAME_INTERVAL: _IOC_TYPE = _IOWR!(b'V', 21, v4l2_subdev_frame_interval);
pub const VIDIOC_SUBDEV_S_FRAME_INTERVAL: _IOC_TYPE = _IOWR!(b'V', 22, v4l2_subdev_frame_interval);
pub const VIDIOC_SUBDEV_ENUM_MBUS_CODE: _IOC_TYPE = _IOWR!(b'V', 2, v4l2_subdev_mbus_code_enum);
pub const VIDIOC_SUBDEV_ENUM_FRAME_SIZE: _IOC_TYPE = _IOWR!(b'V', 74, v4l2_subdev_frame_size_enum);
pub const VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL: _IOC_TYPE = _IOWR!(b'V', 75, v4l2_subdev_frame_interval_enum);
pub const VIDIOC_SUBDEV_G_CROP: _IOC_TYPE = _IOW!(b'V', 59, v4l2_subdev_crop);
pub const VIDIOC_SUBDEV_S_CROP: _IOC_TYPE = _IOR!(b'V', 60, v4l2_subdev_crop);
/* The following ioctls are identical to the ioctls in videodev2.h */
pub const VIDIOC_SUBDEV_G_STD: _IOC_TYPE = _IOR!(b'V', 23, v4l2_std_id);
pub const VIDIOC_SUBDEV_S_STD: _IOC_TYPE = _IOW!(b'V', 24, v4l2_std_id);
pub const VIDIOC_SUBDEV_ENUMSTD: _IOC_TYPE = _IOWR!(b'V', 25, v4l2_standard);
pub const VIDIOC_SUBDEV_G_EDID: _IOC_TYPE = _IOWR!(b'V', 40, v4l2_edid);
pub const VIDIOC_SUBDEV_S_EDID: _IOC_TYPE = _IOWR!(b'V', 41, v4l2_edid);
pub const VIDIOC_SUBDEV_QUERYSTD: _IOC_TYPE = _IOR!(b'V', 63, v4l2_std_id);
pub const VIDIOC_SUBDEV_S_DV_TIMINGS: _IOC_TYPE = _IOWR!(b'V', 87, v4l2_dv_timings);
pub const VIDIOC_SUBDEV_G_DV_TIMINGS: _IOC_TYPE = _IOWR!(b'V', 88, v4l2_dv_timings);
pub const VIDIOC_SUBDEV_ENUM_DV_TIMINGS: _IOC_TYPE = _IOWR!(b'V', 98, v4l2_enum_dv_timings);
pub const VIDIOC_SUBDEV_QUERY_DV_TIMINGS: _IOC_TYPE = _IOR!(b'V', 99, v4l2_dv_timings);
pub const VIDIOC_SUBDEV_DV_TIMINGS_CAP: _IOC_TYPE = _IOWR!(b'V', 100, v4l2_dv_timings_cap);
/*
pub const VIDIOC_SUBDEV_G_ROUTING: _IOC_TYPE = _IOR!(b'V', 38, v4l2_subdev_routing);
pub const VIDIOC_SUBDEV_S_ROUTING: _IOC_TYPE = _IOWR!(b'V', 39, v4l2_subdev_routing);
*/
