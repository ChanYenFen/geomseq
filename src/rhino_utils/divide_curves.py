"""
Divides (possibly multi-segment) curves into points + a continuous arc-length lookup
+ corner_indices at segment joints. RhinoCommon-dependent; GH input handling is in gh/divide_curves_component.py.
"""

import Rhino.Geometry as rg
import math as m


class DivideCurves():
    TMIN = float(0.0)
    MAX_SEG_LENGTH = 300
    def __init__(self, curves):
        """Initialize the DivideCurves class"""
        self.curves = curves

    def get_item(self, index):
        """Get the curve at the specified index"""
        try:
            return self.curves[index]
        except IndexError:
            print ("get_item: index out of range:%s"%index)

    def set_crv_domain(self, crv):
        """Set the domain of the curve from 0 to its length"""
        t_max = crv.GetLength()
        crv.Domain = rg.Interval(float(self.TMIN),float(t_max))

    def get_divide_count(self, curve, segment_length):
        if segment_length > self.MAX_SEG_LENGTH:
            return 1
        count = max(1, int(m.ceil(curve.GetLength() / segment_length)))
        return count

    def get_parameters(self,crv,count,endInc=True):
        params = list(crv.DivideByCount(count,endInc))
        return params

    def check_closed(self,crv):
        return True if crv.IsClosed else False

    def check_explodable(self, crv):
        tMin = self.TMIN
        t_max = crv.GetLength()
        getNext = True
        count = 0
        dc = rg.Continuity.G2_continuous
        while getNext:
            getNext, t = rg.Curve.GetNextDiscontinuity(crv, dc,tMin,t_max)
            count+=1
            if getNext:
                tMin=t
        if count>0:return True
        else:return False

    def get_curve_segments(self, crv):
        return crv.DuplicateSegments() if self.check_explodable(crv) else [crv]

    def flatten_list(self, l):
        return [item for sublist in l for item in sublist]

    def get_division_pts(self, crv, params):
        division_pts =[crv.PointAt(p) for p in params]
        return division_pts

    def get_division_normals(self, crv, params):
        division_tangents =[crv.TangentAt(p) for p in params]
        division_normals = [rg.Vector3d.CrossProduct(v, rg.Vector3d.ZAxis) for v in division_tangents]
        return division_normals

    def get_overlap(self, pts, overlap_length, join_ends):
        """Evaluate if the overlap_length is set higher than the distant of first and last point"""
        if join_ends: check = rg.Point3d.DistanceTo(pts[1], pts[0])
        else: check = rg.Point3d.DistanceTo(pts[0], pts[-1])
        if overlap_length > check:
            overlap_length = check
            print ("get_overlap: overlap_length can't be higher than %s"%overlap_length)
        if overlap_length <= check:
            if join_ends: dir = pts[1]-pts[0]
            else      : dir = pts[0]-pts[-1]
            dir.Unitize()
            new_pt = rg.Point3d.Add(pts[-1], dir*overlap_length)
            pts.append(new_pt)


def _dedupe_segment_params(params, is_last, is_single_segment, is_closed, join_ends):
    """Drops the point that would duplicate the next segment's start, except where a closed curve's own closing point is wanted."""
    if is_single_segment:
        if is_closed and join_ends:
            params.append(params[0])
        return params
    if is_last:
        if is_closed and not join_ends:
            params.pop()
        return params
    params.pop()
    return params


def process_curve(dc, crv, seg_length, join_ends, overlap, overlap_length):
    """Divides one (possibly multi-segment) curve into points + a continuous arc-length lookup + segment-joint corner indices."""
    segs = dc.get_curve_segments(crv)
    is_closed = dc.check_closed(crv)

    crv_pts = []
    crv_lookups = []
    crv_corners = []
    offset = 0.0  # cumulative arc-length of segments already processed

    for j, seg in enumerate(segs):
        dc.set_crv_domain(seg)
        this_seg_length = seg.GetLength()  # unaffected by set_crv_domain
        count = dc.get_divide_count(seg, seg_length)
        params = dc.get_parameters(seg, count, True)
        params = _dedupe_segment_params(params, j == len(segs) - 1, len(segs) == 1, is_closed, join_ends)

        crv_pts.extend(dc.get_division_pts(seg, params))

        global_params = [p + offset for p in params]  # shift into the whole curve's arc-length frame
        if j > 0:
            crv_corners.append(len(crv_lookups))  # this segment's first point is the joint with the previous one
        crv_lookups.extend(global_params)
        offset += this_seg_length

    if overlap and is_closed:
        dc.get_overlap(crv_pts, overlap_length, join_ends)

    return crv_pts, crv_lookups, crv_corners
