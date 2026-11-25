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
