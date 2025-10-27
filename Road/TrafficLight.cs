/*
 *  CityTrafficSimulator - a tool to simulate traffic in urban areas and on intersections
 *  Copyright (C) 2005-2014, Christian Schulte zu Berge
 *  
 *  This program is free software; you can redistribute it and/or modify it under the 
 *  terms of the GNU General Public License as published by the Free Software 
 *  Foundation; either version 3 of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT ANY 
 *  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
 *  PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with this 
 *  program; if not, see <http://www.gnu.org/licenses/>.
 * 
 *  Web:  http://www.cszb.net
 *  Mail: software@cszb.net
 */

using System;
using System.Collections.Generic;
using System.Xml.Serialization;


namespace SimTMDG.Road
{
    /// <summary>
    /// Kapselt eine Ampel und implementiert ein TimelineEntry
    /// </summary>
    [Serializable]
    public class TrafficLight // : TimelineEntry, ISavable
    {
        public enum State
        {
            GREEN,
            RED
        }

        private State _trafficLightState;
        
        [XmlIgnore]
        public State trafficLightState
        {
            get { return _trafficLightState; }
            set { _trafficLightState = value; }
        }

        [XmlIgnore]
        private List<Node> _assignedNodes = new List<Node>();

        [XmlIgnore]
        public List<Node> assignedNodes
        {
            get { return _assignedNodes; }
        }

        #region Hashcodes
        [XmlIgnore]
        private static int hashcodeIndex = 0;

        public int hashcode = -1;

        public override int GetHashCode()
        {
            return hashcode;
        }

        public static void ResetHashcodeIndex()
        {
            hashcodeIndex = 0;
        }

        #endregion

        #region Konstruktoren

        public TrafficLight()
        {
            hashcode = hashcodeIndex++;

            // Initial Event anlegen
            trafficLightState = State.RED;
        }
        #endregion


        #region Speichern/Laden
        ///// <summary>
        ///// DEPRECATED: Hash des Elternknotens (wird für Serialisierung gebraucht)
        ///// </summary>
        //[XmlIgnore]
        //public int parentNodeHash = 0;

        ///// <summary>
        ///// Hashes der zugeordneten LineNodes
        ///// </summary>
        //public List<int> assignedNodesHashes = new List<int>();

        ///// <summary>
        ///// bereitet das TrafficLight auf die XML-Serialisierung vor.
        ///// </summary>
        //public override void PrepareForSave()
        //{
        //    base.PrepareForSave();

        //    assignedNodesHashes.Clear();
        //    foreach (LineNode ln in _assignedNodes)
        //    {
        //        assignedNodesHashes.Add(ln.GetHashCode());
        //    }
        //}
        ///// <summary>
        ///// stellt das TrafficLight nach einer XML-Deserialisierung wieder her
        ///// </summary>
        ///// <param name="saveVersion">Version der gespeicherten Datei</param>
        ///// <param name="nodesList">Liste von allen existierenden LineNodes</param>
        //public override void RecoverFromLoad(int saveVersion, List<LineNode> nodesList)
        //{
        //    // Klassenvariable für Hashcode erhöhen um Kollisionen für zukünftige LineNodes zu verhindern
        //    if (hashcodeIndex <= hashcode)
        //    {
        //        hashcodeIndex = hashcode + 1;
        //    }

        //    // erstmal EventActions setzen
        //    this.defaultAction = SwitchToRed;
        //    foreach (TimelineEvent e in events)
        //    {
        //        e.eventStartAction = SwitchToGreen;
        //        e.eventEndAction = SwitchToRed;
        //    }

        //    // nun die assignedNodes aus der nodesList dereferenzieren
        //    foreach (int hash in assignedNodesHashes)
        //    {
        //        foreach (LineNode ln in nodesList)
        //        {
        //            if (ln.GetHashCode() == hash)
        //            {
        //                _assignedNodes.Add(ln);
        //                ln.tLight = this;
        //                break;
        //            }
        //        }
        //    }

        //    // Alte Versionen konnten nur einen Node pro TrafficLight haben und waren daher anders referenziert, auch darum wollen wir uns kümmern:
        //    if (saveVersion <= 2)
        //    {
        //        foreach (LineNode ln in nodesList)
        //        {
        //            if (ln.GetHashCode() == parentNodeHash)
        //            {
        //                AddAssignedLineNode(ln);
        //                break;
        //            }
        //        }
        //    }
        //}
        #endregion

        /// <summary>
        /// meldet den LineNode ln bei diesem TrafficLight an, sodass es weiß das es diesem zugeordnet ist
        /// </summary>
        /// <param name="ln">anzumeldender LineNode</param>
        public void AddAssignedLineNode(Node ln)
        {
            _assignedNodes.Add(ln);
            ln.tLight = this;
        }

        /// <summary>
        /// meldet den LineNode ln bei diesem TrafficLight wieder ab, sodass es weiß, dass es diesem nicht mehr zugeordnet ist
        /// </summary>
        /// <param name="ln">abzumeldender LineNode</param>
        /// <returns>true, falls der Abmeldevorgang erfolgreich, sonst false</returns>
        public bool RemoveAssignedLineNode(Node ln)
        {
            if (ln != null)
            {
                ln.tLight = null;
                return _assignedNodes.Remove(ln);
            }
            return false;
        }

        /// <summary>
        /// stellt die Ampel auf grün
        /// </summary>
        public void SwitchToGreen()
        {
            this.trafficLightState = State.GREEN;
        }
        /// <summary>
        /// stellt die Ampel auf rot
        /// </summary>
        public void SwitchToRed()
        {
            this.trafficLightState = State.RED;
        }


        ///// <summary>
        ///// meldet das TrafficLight bei den zugeordneten LineNodes ab, sodas das TrafficLight gefahrlos gelöscht werden kann.
        ///// </summary>
        //public override void Dispose()
        //{
        //    base.Dispose();

        //    while (_assignedNodes.Count > 0)
        //    {
        //        RemoveAssignedLineNode(_assignedNodes[0]);
        //    }
        //}

    }
}
