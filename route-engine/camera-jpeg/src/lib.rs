//! Decode a race camera frame into its existing 640 × 480 BGR buffer.
use zune_core::{bytestream::ZCursor, colorspace::ColorSpace, options::DecoderOptions};
use zune_jpeg::JpegDecoder;

// Keep the concrete decoder instantiation in the camera crate. The K1 build
// can tune it independently of the caller, including under ThinLTO.
#[inline(never)]
pub fn decode_bgr_640x480(frame: &[u8], pixels: &mut [u8]) -> bool {
    let options = DecoderOptions::default()
        .jpeg_set_out_colorspace(ColorSpace::BGR)
        .set_max_width(640)
        .set_max_height(480);
    let mut decoder = JpegDecoder::new_with_options(ZCursor::new(frame), options);
    decoder.decode_headers().is_ok()
        && decoder.dimensions() == Some((640, 480))
        && decoder.decode_into(pixels).is_ok()
}
