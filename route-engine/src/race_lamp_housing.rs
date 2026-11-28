//! Image-space lower edge of the dark three-lens backplate. No colour votes and no
//! conversion from pixels to metres: pitch and different housings make that ambiguous.
pub fn lip(bgr: &[u8], w: usize, h: usize) -> Option<f64> {
    if w<64 || h<48 || w.checked_mul(h)?.checked_mul(3)?!=bgr.len() {return None}
    let step=(w/160).max(1);
    let rgb=|x:usize,y:usize| {let i=(y*w+x)*3;(i32::from(bgr[i+2]),i32::from(bgr[i+1]),i32::from(bgr[i]))};
    let lum=|x,y| {let (r,g,b)=rgb(x,y);(2*r+4*g+b)/7};
    // The amber centre is useful for identifying the fixture even when its ends are
    // dark or clipped. A plain black door/cable with a horizontal edge has no marker.
    let mut markers=[None;32];
    for y in (0..h*2/5).step_by(step) {
        let mut n=0;
        for x in (0..=w.div_ceil(step)).map(|i|i*step) {
            let (r,g,b)=if x<w {rgb(x,y)} else {(0,0,0)};
            if r>=76 && 100*g>=32*r && 100*g<83*r && 3*b<2*g && r-b>=48 {n+=1;continue}
            let d=n*step;
            if d>=w/23 && d<=w/5 {
                let cx=x-d/2;let slot=&mut markers[(10*y/h).min(3)*8+(8*cx/w).min(7)];
                if slot.is_none_or(|(_,_,s)|d>s){*slot=Some((cx,y,d))}
            }
            n=0;
        }
    }
    let mut best=None;
    for (cx,cy,d) in markers.into_iter().flatten() {
    // Dark backing beside the coloured marker, not an orange object on the sky.
    if ![-0.7,0.7].iter().all(|s| {let x=cx as f64+s*d as f64;x>=0. && x<w as f64 && lum(x as usize,cy)<115}) {continue}
    let mut edges=[0usize;7];let mut count=0;
    for mult in [-1.45,-0.95,-0.62,0.,0.62,0.95,1.45] {
        let x=(cx as f64+mult*d as f64).round() as isize;
        if x<0 || x>=w as isize {continue}
        let x=x as usize;let sample=(d/14).max(2);
        let end=(cy+2*d).min(h/2).min(h-sample-1);
        let mut scored=None;
        for y in ((cy+d/4).max(2*sample)..end).step_by(step.max(2)/2) {
            let a=(lum(x,y-sample)+lum(x,y-2*sample))/2;let b=lum(x,y+sample);
            if a<105 && b-a>30 && scored.is_none_or(|(s,_)|b-a>s) {scored=Some((b-a,y));}
        }
        if let Some((_,y))=scored {edges[count]=y;count+=1}
    }
    if count<4 {continue}
    edges[..count].sort_unstable();let mid=edges[count/2];
    let agree=edges[..count].iter().filter(|y|y.abs_diff(mid)<=d/3).count();
    let score=agree*d;
    if agree>=4 && best.is_none_or(|(s,_)|score>s) {best=Some((score,mid));}
    }
    best.map(|(_,y)|y as f64/h as f64)
}

use crate::race_lamp_range::Edges;

/// Fit the actual long edges between the lenses. Rows are transported back to
/// the amber centre along the observed tilt before checking column agreement.
pub(super) fn metric_edges(
    frame: &super::Frame<'_>,
    marker: (f64, f64),
    spacing: (f64, f64),
    diameter: f64,
) -> Option<Edges> {
    if diameter < 6.0 || spacing.0 <= 0.0 {
        return None;
    }
    let (cx, cy) = marker;
    let sample = (diameter / 14.0).round().clamp(1.0, 5.0) as i32;
    let luma = |x: usize, y: i32| {
        let (b, g, r) = frame.pixel(x, y as usize);
        (i32::from(r) * 2 + i32::from(g) * 4 + i32::from(b)) / 7
    };
    let fit = |sign: i32| {
        let mut edges = [(0.0, 0.0); 6];
        let mut n = 0;
        for f in [-0.67, -0.51, -0.36, 0.36, 0.51, 0.67] {
            let ux = (cx + f * spacing.0).round() as i32;
            if ux < 1 || ux >= frame.width as i32 - 1 {
                continue;
            }
            let mid = cy + f * spacing.1;
            let x = ux as usize;
            let mut best = None;
            for k in (0.53 * diameter).ceil() as i32..=(1.70 * diameter).ceil() as i32 {
                let y = mid.round() as i32 + sign * k;
                if y < 2 * sample + 1 || y >= frame.height as i32 - 2 * sample - 1 {
                    continue;
                }
                let inside = (luma(x, y - sign * sample) + luma(x, y - sign * 2 * sample)) / 2;
                let outside = (luma(x, y + sign * sample) + luma(x, y + sign * 2 * sample)) / 2;
                let contrast = outside - inside;
                if inside < 130 && contrast >= 15 && best.is_none_or(|(s, _)| contrast > s) {
                    best = Some((contrast, y));
                }
                // The silver crossbar often has a second, sharper border against
                // the sky. Stop at the first sustained transition out of the black case.
                if inside < 110
                    && outside >= 145
                    && contrast > 48
                    && y + sign * 3 * sample > 0
                    && y + sign * 3 * sample < frame.height as i32
                    && luma(x, y + sign * 3 * sample) > 135
                {
                    break;
                }
            }
            if let Some((_, y)) = best {
                edges[n] = (f, y as f64);
                n += 1;
            }
        }
        if n < 4 {
            return None;
        }
        // The illuminated parts of the lenses need not share a centre line.
        // Fit the tilt of the measured black edge itself.
        let mut slopes = [0.0; 15];
        let mut ns = 0;
        for i in 0..n {
            for &(x, y) in &edges[..i] {
                let dx = edges[i].0 - x;
                if dx > 0.30 {
                    slopes[ns] = (edges[i].1 - y) / dx;
                    ns += 1;
                }
            }
        }
        if ns == 0 {
            return None;
        }
        slopes[..ns].sort_unstable_by(f64::total_cmp);
        let tilt = slopes[ns / 2];
        if tilt.abs() > 0.45 * spacing.0 {
            return None;
        }
        let mut ys = [0.0; 6];
        for (i, &(x, y)) in edges[..n].iter().enumerate() {
            ys[i] = y - tilt * x;
        }
        ys[..n].sort_unstable_by(f64::total_cmp);
        let middle = (ys[(n - 1) / 2] + ys[n / 2]) * 0.5;
        let agree = ys[..n]
            .iter()
            .filter(|&&v| (v - middle).abs() <= 2.0 + 0.10 * diameter)
            .count();
        (agree >= 4).then_some(middle * 480.0 / frame.height as f64)
    };
    let top = fit(-1);
    let bottom = fit(1);
    (top.is_some() || bottom.is_some()).then_some(Edges {
        u: cx * 640.0 / frame.width as f64,
        top,
        bottom,
    })
}
