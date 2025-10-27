using SimTMDG.Road;
using SimTMDG.Tools;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;

namespace SimTMDG.Vehicle
{
    [Serializable]
    public class IVehicle : IDM
    {
        #region Hashcodes von Vehicles
        /*
		 * Wir benötigen für Fahrzeuge Hashcodes zur schnellen quasi-eindeutigen Identifizierung. Da sich der Zustand eines Fahrzeuges quasi
		 * ständig ändert, gibt es leider keine zuverlässigen Felder die zur Hashwertberechnung dienen können.
		 * 
		 * Also bedienen wir uns einen alten, nicht umbedingt hübschen, aber bewährten Tricks:
		 * IVehicle verwaltet eine statische Klassenvariable hashcodeIndex, die mit jeder Instanziierung eines Vehicles inkrementiert wird und 
		 * als eindeutiger Hashcode für das Fahrzeug dient. Es muss insbesondere sichergestellt werden, dass bei jeder abgeleiteten Klasse entweder
		 * der Elternkonstruktor aufgerufen wird, oder sich die abgeleitete Klasse selbst um einen gültigen Hashcode kümmert.
		 */

        /// <summary>
        /// Klassenvariable welche den letzten vergebenen hashcode speichert und bei jeder Instanziierung eines Objektes inkrementiert werden muss
        /// </summary>
        private static int hashcodeIndex = 0;

        /// <summary>
        /// Hashcode des instanziierten Objektes
        /// </summary>
        private int hashcode = -1;

        /// <summary>
        /// gibt den Hashcode des Fahrzeuges zurück.
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            return hashcode;
        }

        /// <summary>
        /// Standardkonstruktor, der nichts tut als den Hashcode zu setzen.
        /// </summary>
        public IVehicle()
        {
            hashcode = hashcodeIndex++;
            dumb = false;
            //_statistics.startTime = GlobalTime.Instance.currentTime;
            //_statistics.startTimeOnNodeConnection = GlobalTime.Instance.currentTime;
        }

        public IVehicle(RoadSegment cs, int laneIndex, Color c, List<RoadSegment> r)
        {
            hashcode = hashcodeIndex++;
            _state.currentSegment = cs;
            _state.laneIdx = laneIndex;
            color = c;
            Routing = new Routing();
            for(int i = 0; i < r.Count; i++)
            {
                Routing.Push(r[i]);
            }

            Random rnd = new Random();
            _physics = new IVehicle.Physics(14, 14, 0);

            dumb = false;

            newCoord();
            RotateVehicle(currentSegment.startNode, currentSegment.endNode);
        }

        #endregion

        public long StopSegmentId { get; protected set; }
        public bool IsQueued { get; set; } = false;

        /// <summary>
        /// Flag untuk menandai kendaraan baru saja dilepas agar tidak langsung mengantri lagi.
        /// </summary>
        public bool IsReleased { get; set; } = false;

        #region Physics

        /// <summary>
        /// Physik des Fahrzeuges
        /// </summary>
        public IVehicle.Physics _physics; // PROTECTED
        /// <summary>
        /// Physik des Fahrzeuges
        /// </summary>
        public IVehicle.Physics physics
        {
            get { return _physics; }
            set { _physics = value; }
        }

        /// <summary>
        /// Struktur, die Wunschgeschwindigkeit, Geschwindigkeit und Beschleunigung kapselt
        /// </summary>
        public struct Physics
        {
            /// <summary>
            /// Erstellt ein neues Physics Record
            /// </summary>
            /// <param name="d">Wunschgeschwindigkeit</param>
            /// <param name="v">Geschwindigkeit</param>
            /// <param name="a">Beschleunigung</param>
            public Physics(double d, double v, double a)
            {
                targetVelocity = d;
                velocity = v;
                acceleration = a;
                multiplierTargetVelocity = 1;
            }

            /// <summary>
            /// gewünschte Gecshwindigkeit des Autos 
            /// </summary>
            public double targetVelocity;

            /// <summary>
            /// Geschwindigkeit des Fahrzeuges
            /// (sehr wahrscheinlich gemessen in Änderung der Position/Tick)
            /// </summary>
            public double velocity;

            /// <summary>
            /// Beschleunigung des Fahrzeugens
            /// (gemessen in Änderung der Geschwindigkeit/Tick)
            /// </summary>
            public double acceleration;

            /// <summary>
            /// Multiplikator für die Wunschgeschwindigkeit.
            /// Kann benutzt werden, um kurzzeitig schneller zu fahren (etwa um einen Spurwechsel zu machen)
            /// </summary>
            public double multiplierTargetVelocity;
        }

        /// <summary>
		/// Target Velocity of this vehicle.
		/// Currently only a shortcut to the target velocity of the current NodeConnection of this vehicle.
		/// </summary>
		public double targetVelocity
        {
            get
            {
                //return (routing.Route != null) ? Math.Min(_physics.targetVelocity, currentNodeConnection.targetVelocity) : _physics.targetVelocity;
                return _physics.targetVelocity;
            }
        }

        /// <summary>
        /// effektive gewünschte Gecshwindigkeit des Autos (mit Multiplikator multipliziert)
        /// </summary>
        public double effectiveDesiredVelocity
        {
            get { return targetVelocity * _physics.multiplierTargetVelocity; }
        }

        #endregion


        #region State
        /// <summary>
        /// Statusrecord, kapselt aktuelle NodeConnection+Position
        /// </summary>
        public struct State
        {
            /// <summary>
            /// Konstruktor, der nur die Position initialisiert. Der Rest ist uninitialisiert!
            /// </summary>
            /// <param name="p">Zeitposition auf der Linie</param>
            public State(double p)
            {
                currentSegment = null;
                laneIdx = -1;
                targetLaneIdx = -1;
                prevLaneIdx = -1;
                m_Position = p;
                _freeDrive = true;
            }

            /// <summary>
            /// Standardkonstruktor, benötigt eine Nodeconnection und Zeitposition. Der Rest wird intern berechnet
            /// </summary>
            /// <param name="nc">aktuelle NodeConnection</param>
            /// <param name="p">Zeitposition auf nc</param>
            public State(RoadSegment cs, int laneIndex, double p)
            {
                currentSegment = cs;
                laneIdx = laneIndex;
                targetLaneIdx = -1;
                prevLaneIdx = -1;
                m_Position = p;
                _freeDrive = true;
            }

            /// <summary>
            /// die Line, auf der sich das Fahrzeug gerade befindet
            /// </summary>
            public RoadSegment currentSegment;

            /// <summary>
            /// Lane index in current RoadSegment
            /// </summary>
            public int laneIdx;

            /// <summary>
            /// Target lane index in current RoadSegment
            /// </summary>
            public int targetLaneIdx;

            /// <summary>
            /// Previous lane index in current RoadSegment
            /// </summary>
            public int prevLaneIdx;

            /// <summary>
            /// relative Position auf der Line
            /// </summary>
            private double m_Position;
            /// <summary>
            /// Zeitposition auf der Linie
            /// </summary>
            public double position
            {
                get { return m_Position; }
                set
                {
                    m_Position = value;
                    if (currentSegment != null)
                    {
                        //double t = currentNodeConnection.lineSegment.PosToTime(m_Position);
                        //m_PositionAbs = newCoord();
                        //m_Orientation = currentNodeConnection.lineSegment.DerivateAtTime(t);
                    }
                }
            }
            public bool _freeDrive;
        }


        /// <summary>
		/// aktueller State des Fahrezeuges
		/// </summary>
        protected IVehicle.State _state;

        /// <summary>
        /// aktueller State des Fahrezeuges
        /// </summary>
        public IVehicle.State state
        {
            get { return _state; }
            set { _state = value; }
        }
        

        /// <summary>
        /// aktuelle Bogenlängenposition auf der aktuellen NodeConnection
        /// </summary>
        public double currentPosition
        {
            get { return state.position; }
        }

        /// <summary>
        /// aktuelle NodeConnection
        /// </summary>
        public RoadSegment currentSegment
        {
            get { return state.currentSegment; }
        }

        #endregion

        public int vehiclesIndex = 0;
        public Double length = 7;
        public Double width = 3;
        public Double distance = 0.0;
        private Double rearPos;
        private Double frontPos;
        public Double orientation;
        public Color color = Color.Black;
        public Vector2 absCoord = new Vector2();
        public Double rotation = 0;
        private bool alreadyMoved = false;

        public double RearPos
        {
            get
            {
                return distance - (length / 2);
            }

            set
            {
                rearPos = value;
            }
        }

        public double FrontPos
        {
            get
            {
                return distance + (length / 2);
            }

            set
            {
                frontPos = value;
            }
        }



        #region draw
        protected virtual GraphicsPath BuildGraphicsPath()
        {
            GraphicsPath toReturn = new GraphicsPath();
            //Vector2 direction = state.orientation;
            //Vector2 orientation = direction.Normalized;
            //Vector2 normal = direction.RotatedClockwise.Normalized;

            PointF normal = new PointF((float)absCoord.X, (float)absCoord.Y);

            PointF[] ppoints =
                {                        
                    RotatePoint(new PointF((float)(absCoord.X + (length / 2)), (float)(absCoord.Y + (width / 2))), normal, rotation),
                    RotatePoint(new PointF((float)(absCoord.X + (length / 2)), (float)(absCoord.Y - (width / 2))), normal, rotation),
                    RotatePoint(new PointF((float)(absCoord.X - (length / 2)), (float)(absCoord.Y - (width / 2))) , normal, rotation),
                    RotatePoint(new PointF((float)(absCoord.X - (length / 2)), (float)(absCoord.Y + (width / 2))) , normal, rotation)                       
                };
            
            toReturn.AddPolygon(ppoints);

            return toReturn;
        }

        SolidBrush fillBrush = new SolidBrush(Color.Black);

        public virtual void Draw(Graphics g)
        {
            GraphicsPath gp = BuildGraphicsPath();
            
            fillBrush.Color = color;
            g.FillPath(fillBrush, gp);
            //Font debugFont = new Font("Calibri", 6);
            //Brush blackBrush = new SolidBrush(Color.Black);
            //g.DrawString(hashcode.ToString(), debugFont, blackBrush, this.absCoord);

            //g.DrawString(hashcode.ToString() + " @ " + currentPosition.ToString("####") + "dm - " + physics.velocity.ToString("##.#") + "m/s - Mult.: ", debugFont, blackBrush, absCoord + new Vector2(0, -10));
        }

        #endregion

        #region route
        private Routing routing;

        internal Routing Routing
        {
            get { return routing; }

            set { routing = value; }
        }

        //internal WaySegment CurrentSegment
        //{
        //    get
        //    {
        //        return currentSegment;
        //    }

        //    set
        //    {
        //        currentSegment = value;
        //    }
        //}



        #endregion

        #region temp_Dumb
        public bool dumb;
        #endregion

        #region think
        public void Think(double tickLength)
        {
            List<RoadSegment> route = new List<RoadSegment>();
            foreach (RoadSegment ws in routing.Route)
                route.Add(ws);

            double acceleration;
            if (!dumb)
            {
                acceleration = Think(route, tickLength);
            } else { acceleration = 0; }
            
            Accelerate(acceleration);
        }

        Boolean thinkAboutLineChange = false;
        VehicleDistance leadVd;

        public double Think(List<RoadSegment> route, double tickLength)
        {
            if (currentSegment == null || route == null || route.Count == 0 || this.IsQueued)
            {
                if (this.IsQueued || currentSegment == null) _physics.acceleration = 0;
                return 0;
            }

            double lookaheadDistance = 300.0;
            double lowestAcceleration;

            #region Vehicle in Front Calculation (IDM Core)
            leadVd = findVehicleInFront(route);

            double deltaV = 0;
            double effectiveDistance;

            if (leadVd != null && leadVd.vehicle != null)
            {
                effectiveDistance = leadVd.distance;
                deltaV = this.physics.velocity - leadVd.vehicle._physics.velocity;

                if (effectiveDistance < 0.1)
                {
                    effectiveDistance = 0.1;
                    System.Diagnostics.Debug.WriteLine($"Warning: Vehicle {this.GetHashCode()} detected near zero/negative gap ({leadVd.distance:F2}) to {leadVd.vehicle.GetHashCode()}. Clamping to 0.1.");
                }
            }
            else
            {
                effectiveDistance = lookaheadDistance;
                deltaV = this.physics.velocity;
            }

            lowestAcceleration = CalculateAcceleration(this.physics.velocity, effectiveDesiredVelocity, effectiveDistance, deltaV);
            System.Diagnostics.Debug.WriteLine($"Vehicle {this.GetHashCode()} IDM Calc: v={this.physics.velocity:F2}, v0={effectiveDesiredVelocity:F2}, s={effectiveDistance:F2}, dV={deltaV:F2} -> Accel={lowestAcceleration:F2}");


            #endregion

            #region Nudge Logic (jika v ≈ 0 dan ada ruang)
            double requiredGapForNudge = s0 + 1.5;
            if (this.physics.velocity < 0.1 && lowestAcceleration <= 0.01 && effectiveDistance > requiredGapForNudge)
            {
                System.Diagnostics.Debug.WriteLine($"Vehicle {this.GetHashCode()} NUDGED. v={physics.velocity:F2}, IDM_accel={lowestAcceleration:F2}, gap={effectiveDistance:F2} > req={requiredGapForNudge:F2}");
                lowestAcceleration = 0.15 * a;
            }
            #endregion

            #region Final Acceleration Limiting
            lowestAcceleration = Math.Max(lowestAcceleration, -this.b * 3.0);
            lowestAcceleration = Math.Min(lowestAcceleration, this.a);
            #endregion

            System.Diagnostics.Debug.WriteLine($"Vehicle {this.GetHashCode()} Think Final Accel: {lowestAcceleration:F2}");
            return lowestAcceleration;
        }


        private double ConsiderLaneChange(int direction, double tickLength, List<RoadSegment> route, double lowestAcceleration)
        {
            #region consider lane change
            VehicleDistance newLeadVd = findVehicleInFront(route, direction);
            VehicleDistance newLagVd = findVehicleInBehind(direction);
            VehicleDistance lagVd = findVehicleInBehind();

            double sNewLead;
            double sNewLag;
            double sLead;
            double sLag;
            double vNewLead;
            double vNewLag;
            double vLead;
            double vLag;
            double accNew;
            double accNewLag;
            double newAccNewLag;
            double accLag;
            double newAccOldLag;

            if (newLeadVd != null)
            {
                sNewLead = newLeadVd.distance;
                vNewLead = newLeadVd.vehicle.physics.velocity;
            }
            else
            {
                sNewLead = 768;
                vNewLead = physics.velocity;
            }

            if (newLagVd != null)
            {
                sNewLag = newLagVd.distance;
                vNewLag = newLagVd.vehicle.physics.velocity;
                accNewLag = newLagVd.vehicle.physics.acceleration;
            }
            else
            {
                sNewLag = 768;
                vNewLag = 0;
                accNewLag = 0;
            }

            if (lagVd != null)
            {
                sLag = lagVd.distance;
                vLag = lagVd.vehicle.physics.velocity;
                accLag = lagVd.vehicle.physics.acceleration;
            }
            else
            {
                sLag = 768;
                vLag = 0;
                accLag = 0;
            }

            if (leadVd != null)
            {
                sLead = leadVd.distance;
                vLead = leadVd.vehicle.physics.velocity;
            }
            else
            {
                sLead = 768;
                vLead = 0;
            }

            accNew = CalculateAcceleration(physics.velocity, effectiveDesiredVelocity, sNewLead, physics.velocity - vNewLead);
            newAccNewLag = CalculateAcceleration(vNewLag, effectiveDesiredVelocity, sNewLag, vNewLag - physics.velocity);

            newAccOldLag = CalculateAcceleration(
                vLag,
                effectiveDesiredVelocity,
                sLag + sLead + this.length,
                vLag - vLead
                );


            if (SafetyCriterion(newAccNewLag))
            {
                if (sNewLead > 0 && sNewLag > 0 && LaneChangeDecision(accNew, physics.acceleration, newAccNewLag, accNewLag, newAccOldLag, accLag))
                {
                    thinkAboutLineChange = true;
                    doLaneChange(direction, tickLength);
                    lowestAcceleration = accNew;
                }
            }
            #endregion

            return lowestAcceleration;
        }


        private void doLaneChange(int direction, double tickLength)
        {
            _state.targetLaneIdx = state.laneIdx + direction;
            currentLane(direction).vehicles.Add(this);
            currentLane().vehToRemove.Add(this);

            _state.prevLaneIdx = state.laneIdx;
            _state.laneIdx = state.targetLaneIdx;
            _state.targetLaneIdx = -1;

            resetDelay(tickLength);
        }

        private double FINITE_LANE_CHANGE_TIME_S = 4;
        private double tLaneChangeDelay = 0;

        private Boolean inProcessOfLaneChange()
        {
            return (tLaneChangeDelay > 0 && tLaneChangeDelay < FINITE_LANE_CHANGE_TIME_S);
        }

        private void resetDelay(double dt)
        {
            tLaneChangeDelay = 0;
        }

        public void updateLaneChangeDelay(double dt)
        {
            tLaneChangeDelay += dt;
        }


        private VehicleDistance findVehicleInFront(List<RoadSegment> route, int direction = 0)
        {
            VehicleDistance vd = null;
            double searchedDistance = 0;

            var lane = currentLane(direction);
            if (lane != null && lane.vehicles.Count > 0 &&
                vehiclesIndex >= 0 && vehiclesIndex < lane.vehicles.Count - 1)
            {
                var front = lane.vehicles[vehiclesIndex + 1];
                double returnDistance = front.distance - (front.length / 2 + this.distance + this.length / 2);
                vd = new VehicleDistance(front, returnDistance);
            }
            else
            {
                if (route.Count == 0) return null;

                searchedDistance += route[0].Length - this.FrontPos;
                if (searchedDistance < 768)
                {
                    for (int i = 1; i < route.Count; i++)
                    {
                        int laneIdx = state.laneIdx + direction;
                        if (laneIdx < 0 || laneIdx >= route[i].lanes.Count) continue;

                        var nextLane = route[i].lanes[laneIdx];
                        if (nextLane.vehicles.Count > 0)
                        {
                            var front = nextLane.vehicles[0];
                            double returnDistance = (front.distance - front.length / 2) + searchedDistance;
                            vd = new VehicleDistance(front, returnDistance);
                            break;
                        }
                        else
                        {
                            searchedDistance += route[i].Length;
                        }
                    }
                }
            }

            return vd;
        }


        private VehicleDistance findVehicleInFront()
        {
            VehicleDistance vd = null;
            if (this.currentLane().vehicles.Count > 1)
            {
                for (int i = 0; i < this.currentLane().vehicles.Count; i++)
                {
                    if (this.currentLane().vehicles[i].distance > this.distance)
                    {
                        if (vd == null)
                        {
                            vd = new VehicleDistance(this.currentLane().vehicles[i],
                                    this.currentLane().vehicles[i].distance 
                                    - (this.distance + (this.currentLane().vehicles[i].length / 2) + (this.length / 2))
                                );
                        }else if (this.currentLane().vehicles[i].distance < vd.vehicle.distance)
                        {
                            vd.vehicle = this.currentLane().vehicles[i];
                            vd.distance = this.currentLane().vehicles[i].distance 
                                - (this.distance + (this.currentLane().vehicles[i].length / 2) + (this.length / 2));
                        }
                    }
                }
            }

            if (vd == null)
            {
                if (currentLane().vehicles.Count > 1)
                {
                    for (int i = 0; i < currentLane().vehicles.Count; i++)
                    {
                        if (currentLane().vehicles[i].distance > this.distance)
                        {
                            if (vd == null)
                            {
                                double distanceToFront = (this.currentSegment.Length - (this.distance + (this.length / 2)))
                                                         + (currentLane().vehicles[i].distance
                                                         - (currentLane().vehicles[i].length / 2));

                                vd = new VehicleDistance(currentLane().vehicles[i], distanceToFront);
                            }
                            else if (currentLane().vehicles[i].distance < vd.vehicle.distance)
                            {
                                double distanceToFront = (this.currentSegment.Length - (this.distance + (this.length / 2)))
                                                         + (currentLane().vehicles[i].distance
                                                         - (currentLane().vehicles[i].length / 2));

                                vd.vehicle = currentLane().vehicles[i];
                                vd.distance = distanceToFront;
                            }
                        }
                    }
                }
            }

            return vd;
        }


        private VehicleDistance findVehicleInBehind(int direction = 0)
        {
            VehicleDistance vd = null;

            if ((currentSegment.lanes.Count + 1 > state.laneIdx + direction) && (state.laneIdx + direction >= 0) && (vehiclesIndex != 0))
            {
                if (currentLane(direction).vehicles.Count > 1)
                {
                    int vdIdx = currentLane(direction).findVehicleBehind(this.distance);
                    if (vdIdx != -1)
                    {
                        vd = new VehicleDistance(currentLane(direction).vehicles[vdIdx], this.RearPos - currentLane(direction).vehicles[vdIdx].FrontPos);
                        return vd;
                    }
                }
            }

            if (currentSegment.prevSegment.Count > 0)
            {
                vd = findVehicleInBehindRecursive(currentSegment, 0, direction);
            }

            return vd;
        }


        private VehicleDistance findVehicleInBehindRecursive(RoadSegment currSegment, double searchedDistance, int direction = 0)
        {
            VehicleDistance vd = null;
            double tempDistance = 768;

            if (searchedDistance > 768)
            {
                return vd;
            }
            else
            {

                for (int i = 0; i < currSegment.prevSegment.Count; i++)                             // Foreach prevSegment
                {
                    if (currSegment.prevSegment[i].lanes.Count > state.laneIdx + direction)         // If there's target lane in prev segment
                    {
                        if (currSegment.prevSegment[i].lanes[state.laneIdx + direction].vehicles.Count > 0)  // If there's a vehicle in target lane in prev segment
                        {
                            tempDistance = this.RearPos +
                                currSegment.prevSegment[i].Length -
                                currSegment.prevSegment[i].lanes[state.laneIdx + direction].vehicles[currSegment.prevSegment[i].lanes[state.laneIdx + direction].vehicles.Count - 1].FrontPos;

                            if ((vd == null)||(tempDistance < vd.distance))                                   // Only save the result if the vehicle is closer
                            {
                                // Return furthest vehicle
                                vd = new VehicleDistance(null, 0);
                                vd.vehicle = currSegment.prevSegment[i].lanes[state.laneIdx + direction].vehicles[currSegment.prevSegment[i].lanes[state.laneIdx + direction].vehicles.Count - 1];
                                vd.distance = tempDistance;
                            }
                        }
                        else                                                                        // Search for prev segment in prev segment :p
                        {
                            searchedDistance += currentSegment.Length;
                            vd = findVehicleInBehindRecursive(currSegment.prevSegment[i], searchedDistance, direction);
                        }
                    }
                }
            }

            return vd;
        }

        /// <summary>
		/// Searches for the next TrafficLight on the vehicle's route within the given distance.
		/// </summary>
		/// <param name="route">Route of the Vehicle.</param>
		/// <param name="arcPos">Current arc position of the vehicle on the first NodeConnection on <paramref name="route"/>.</param>
		/// <param name="distance">Distance to cover during search.</param>
		/// <param name="considerOnlyRed">If true, only red traffic lights will be considered.</param>
		/// <returns>The distance to the next TrafficLight on the vehicle's route that covers the given constraints. <paramref name="distance"/> if no such TrafficLight exists.</returns>
		private double GetDistanceToNextTrafficLightOnRoute(List<RoadSegment> route, double arcPos, double distance, bool considerOnlyRed)
        {
            double toReturn = distance;
            double searchedDistance = 0;

            while(searchedDistance < 768)
            {
                for(int i=0; i < route.Count; i++)
                {
                    if(route[i].endNode.tLight != null)
                    {
                        if (route[i].endNode.tLight.trafficLightState == TrafficLight.State.RED)
                        {
                            toReturn = route[i].Length + searchedDistance - (this.distance + this.length / 2);
                            return toReturn;
                        }
                            
                    }else
                    {
                        searchedDistance += route[i].Length;
                    }
                }

            }

            return toReturn;
        }


        public void Accelerate(double newAcceleration)
        {
            _physics.acceleration = newAcceleration;
        }


        #region move

        public void Move(double tickLength)
        {
            if (!alreadyMoved)
            {
                if (this.IsQueued)
                {
                    _physics.velocity = 0;
                    _physics.acceleration = 0;
                    distance = currentSegment.Length;
                    newCoord();
                    alreadyMoved = true;
                    return;
                }

                _physics.velocity += _physics.acceleration * tickLength;

                if (_physics.velocity < 0)
                    _physics.velocity = 0;

                distance += _physics.velocity * tickLength;

                #region vehicle change segment

                if (distance >= state.currentSegment.Length)
                {
                    if (this.currentSegment.Id == this.StopSegmentId && !this.IsReleased)
                    {
                        this.IsQueued = true;
                        this.distance = state.currentSegment.Length;
                        _physics.velocity = 0;
                        _physics.acceleration = 0;
                        newCoord();
                        alreadyMoved = true;
                        return;
                    }

                    double newDistance = distance - currentSegment.Length;
                    if (Routing.Route.Count > 0)
                    {
                        Routing.Route.RemoveFirst();
                        if (Routing.Route.Count > 0)
                        {
                            if (newDistance > Routing.Route.First.Value.Length)
                            {
                                newDistance = newDistance - Routing.Route.First.Value.Length;
                                Routing.Route.RemoveFirst();
                                this.IsReleased = false;
                                RemoveFromCurrentSegment(Routing.Route.First.Value, newDistance);
                            }
                            else
                            {
                                this.IsReleased = false;
                                RemoveFromCurrentSegment(Routing.Route.First.Value, newDistance);
                            }
                        }
                        else
                        {
                            this.IsReleased = false;
                            RemoveFromCurrentSegment(null, 0);
                        }
                    }
                    else
                    {
                        this.IsReleased = false;
                        RemoveFromCurrentSegment(null, 0);
                    }
                }
                else
                {
                    newCoord();
                }
                #endregion

                alreadyMoved = true;
            }
        }

        public void Reset()
        {
            alreadyMoved = false;
        }
        #endregion

        #endregion

        private void RemoveFromCurrentSegment(RoadSegment nextSegment, double startPosition)
        {
            currentLane().vehToRemove.Add(this);

            if (nextSegment != null)
            {
                int targetLaneIdx = state.laneIdx;

                if (targetLaneIdx >= nextSegment.lanes.Count)
                    targetLaneIdx = nextSegment.lanes.Count - 1;

                if (nextSegment.lanes[targetLaneIdx].vehicles.Count > 0)
                {
                    var lastVehicle = nextSegment.lanes[targetLaneIdx].vehicles.Last();
                    if (Math.Abs(lastVehicle.distance - startPosition) < 5)
                    {
                        currentLane().vehToRemove.Remove(this);
                        return;
                    }
                }

                _state.laneIdx = targetLaneIdx;
                _state.currentSegment = nextSegment;

                distance = startPosition;
                nextSegment.lanes[targetLaneIdx].vehicles.Add(this);

                newCoord();
                RotateVehicle(currentSegment.startNode, currentSegment.endNode);
            }
        }

        public void newCoord()
        {
            Vector2 difference = currentLane().endNode.Position - currentLane().startNode.Position;

            PointF toReturn = new PointF();

            toReturn.X = (float) difference.X * (float) (this.distance / currentLane().Length) +
                (float) currentLane().startNode.Position.X;
            toReturn.Y = (float) difference.Y * (float) (this.distance / currentLane().Length) + 
                (float) currentLane().startNode.Position.Y;


            absCoord = toReturn;
        }

        public PointF RotatePoint(PointF point, PointF origin, Double angle)
        {
            float translated_X = point.X - origin.X;
            float translated_Y = point.Y - origin.Y;
            PointF rotated = new PointF
            {
                X = (float)(translated_X * Math.Cos(angle) - translated_Y * Math.Sin(angle)) + origin.X,
                Y = (float)(translated_X * Math.Sin(angle) + translated_Y * Math.Cos(angle)) + origin.Y
            };
            return rotated;
        }

        public void RotateVehicle(Node startPoint, Node endPoint)
        {
            rotation = Vector2.AngleBetween(startPoint.Position, endPoint.Position);
        }



        private SegmentLane currentLane(int direction = 0)
        {
            return currentSegment.lanes[state.laneIdx + direction];
        }

    }
}
