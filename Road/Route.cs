using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SimTMDG.Road
{
    public class Routing : IEnumerable<RoadSegment>
    {
        /// <summary>
		/// Wegroute
		/// </summary>
		private LinkedList<RoadSegment> route;

        /// <summary>
		/// Kosten der gesamten Route
		/// </summary>
		public double costs;

        internal LinkedList<RoadSegment> Route
        {
            get
            {
                return route;
            }

            set
            {
                route = value;
            }
        }

        #region Constructor
        /// <summary>
        /// Standardkonstruktor, erstellt eine neue leere Wegroute zu einem Zielknoten
        /// </summary>
        public Routing()
        {
            Route = new LinkedList<RoadSegment>();
            costs = 0;
        }
        #endregion

        /// <summary>
		/// Pusht das übergebene RouteSegment auf den route-Stack und aktualisiert die Kosten und Anzahl benötigter Spurwechsel
		/// </summary>
		/// <param name="rs">einzufügendes RouteSegment</param>
		public void Push(RoadSegment ws)
        {
            Route.AddLast(ws);
            costs += ws.Length; // TODO use length as cost for now
        }

        public IEnumerator<RoadSegment> GetEnumerator()
        {
            return Route.GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return Route.GetEnumerator();
        }
    }
}
