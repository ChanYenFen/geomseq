using System;
using Grasshopper.Kernel;

namespace GeomSeq.Components;

/// <summary>
/// Inputs a component hides until asked for: zoom in and press the ⊕ under its last input.
/// They come in a fixed order after the always-shown inputs, so ports keep their indices
/// whichever are shown, and an absent one reads as its default.
/// </summary>
/// <remarks>
/// Grasshopper's variable parameters, used instead of a right-click menu: a menu needs
/// System.Windows.Forms, which this cross-platform net7.0 build cannot reference, and an
/// input can still be wired where a menu setting cannot.
/// </remarks>
internal sealed class OptionalInputs
{
    private readonly int _shown;
    private readonly Func<IGH_Param>[] _make;

    /// <param name="shown">How many inputs are always there.</param>
    /// <param name="make">Factories for the hidden ones, in the order they are added.</param>
    public OptionalInputs(int shown, params Func<IGH_Param>[] make)
    {
        _shown = shown;
        _make = make;
    }

    public bool CanInsert(GH_Component c, GH_ParameterSide side, int index)
        => side == GH_ParameterSide.Input && index == c.Params.Input.Count &&
           index >= _shown && index < _shown + _make.Length;

    public bool CanRemove(GH_Component c, GH_ParameterSide side, int index)
        => side == GH_ParameterSide.Input && index == c.Params.Input.Count - 1 && index >= _shown;

    public IGH_Param Create(int index) => _make[index - _shown]();

    /// <summary>The value of a hidden input, or <paramref name="fallback"/> when it is not shown or empty.</summary>
    public static T Read<T>(GH_Component c, IGH_DataAccess da, string name, T fallback)
    {
        int index = c.Params.IndexOfInputParam(name);
        if (index < 0)
            return fallback;
        T value = fallback;
        return da.GetData(index, ref value) ? value : fallback;
    }

    public static TParam Optional<TParam>(string name, string nick, string description)
        where TParam : IGH_Param, new()
        => new()
        {
            Name = name,
            NickName = nick,
            Description = description,
            Access = GH_ParamAccess.item,
            Optional = true,
        };

    /// <summary>
    /// The plane whose x axis runs along the lines and whose origin sits on the line grid:
    /// <paramref name="plane"/> turned by <paramref name="angle"/> radians about its normal,
    /// then moved <paramref name="phase"/> along its new y axis. In it, line k lies at
    /// y = k * spacing -- what <see cref="ResampleOnLatticeComponent"/> takes as its frame.
    /// </summary>
    public static Rhino.Geometry.Plane Frame(Rhino.Geometry.Plane plane, double angle, double phase)
    {
        Rhino.Geometry.Plane frame = plane;
        frame.Rotate(angle, plane.ZAxis);
        frame.Origin = frame.Origin + frame.YAxis * phase;
        return frame;
    }
}
