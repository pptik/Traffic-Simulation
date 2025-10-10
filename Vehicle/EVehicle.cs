using System.ComponentModel;

namespace SimTMDG.Vehicle
{
    public enum EVehicleType
    {
        [Description("car")]
        CAR,

        [Description("bus")]
        BUS,

        [Description("motorcycle")]
        MOTORCYCLE,

        [Description("truck")]
        TRUCK
    }
}
