using GH_IO.Serialization;
using GH_IO.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Serialization.Formatters.Binary;
using System.Text;
using System.Threading.Tasks;
using Krill.Containers;

namespace Krill.Grasshopper.WritingHelper
{
    internal static class GH_WriterHelper
    {
        public static void Write(GH_IWriter writer, string name, LinearSolution2d linSol)
        {
            Write(writer, name + "bodyload", linSol.bodyload);
            Write(writer, name + "displacments", linSol.displacments);
            Write(writer, name + "mask", linSol.mask);
            Write(writer, name + "principalDirections", linSol.principalDirections);
            Write(writer, name + "springs", linSol.springs);
            Write(writer, name + "utilization", linSol.utilization);
            Write(writer, name + "weighting", linSol.weighting);

            if (linSol.boundaryConditions != null)
            {
                writer.SetInt32(name + "bcCount", linSol.boundaryConditions.Count);
                for (int i = 0; i < linSol.boundaryConditions.Count; i++)
                    Write(writer, name + "bcs" + i.ToString(), linSol.boundaryConditions[i]);
            }
            else
                writer.SetInt32(name + "bcCount", 0);

            writer.SetDouble(name + "peridelta", linSol.peridelta);
            writer.SetDouble(name + "elasticModulus", linSol.elasticModulus);
            writer.SetDouble(name + "bondStiffness", linSol.bondStiffness);
            writer.SetDouble(name + "oldcuttoff", linSol.oldcuttoff);

            writer.SetBoolean(name + "relaxTension", linSol.relaxTension);

            writer.SetByteArray(name + "nList", GetBytes(linSol.nList));
        }

        public static void Write<T>(GH_IWriter writer, string name, Voxels2d<T> voxels)
        {
            if (voxels == null)
                return;

            writer.SetByteArray(name + "cells", GetBytes(voxels.cellValues));
            writer.SetInt32(name + "n", voxels.n);
            writer.SetDouble(name + "delta", voxels.delta);
            writer.SetPoint2D(name + "origin", new GH_Point2D(voxels.origin.X, voxels.origin.Y));
        }

        public static void Write(GH_IWriter writer, string name, BoundaryConditionNuemann2d bc)
        {
            var options = new Rhino.FileIO.SerializationOptions();
            string curve = bc.curve.ToJSON(options);
            writer.SetString(name + "curve", curve);
            writer.SetBoolean(name + "normal", bc.normal);
            writer.SetDoubleArray(name + "load", new double[] { bc.load.X, bc.load.Y, bc.load.Z });
        }

        public static void Write(GH_IWriter writer, string name, BoundaryConditionDirechlet2d bc)
        {
            var options = new Rhino.FileIO.SerializationOptions();
            string curve = bc.curve.ToJSON(options);
            writer.SetString(name + "curve", curve);
            writer.SetBoolean(name + "normal", bc.normal);
            writer.SetDoubleArray(name + "displacement", new double[] { bc.displacement.X, bc.displacement.Y, bc.displacement.Z });
            writer.SetBoolean(name + "lockX", bc.lockX);
            writer.SetBoolean(name + "lockY", bc.lockY);

            // Add the rest
        }

        private static byte[] GetBytes<T>(T obj)
        {
            if (obj == null)
                return null;
            BinaryFormatter bf = new BinaryFormatter();
            using (MemoryStream ms = new MemoryStream())
            {
                bf.Serialize(ms, obj);
                return ms.ToArray();
            }
        }
    }

    internal static class GH_ReaderHelper
    {
        public static LinearSolution2d Reader(GH_IReader reader, string name, LinearSolution2d linSol)
        {
            linSol.bodyload = Reader(reader, name + "bodyload", new Voxels2d<Vector2d>());
            linSol.displacments = Reader(reader, name + "displacments", new Voxels2d<Vector2d>());
            linSol.mask = Reader(reader, name + "mask", new Voxels2d<int>());
            linSol.principalDirections = Reader(reader, name + "principalDirections", new Voxels2d<Vector3d[]>());
            linSol.springs = Reader(reader, name + "springs", new Voxels2d<Vector2d>());
            linSol.utilization = Reader(reader, name + "utilization", new Voxels2d<double>());
            linSol.weighting = Reader(reader, name + "weighting", new Voxels2d<double>());

            int count = reader.GetInt32(name + "bcCount");
            linSol.boundaryConditions = new List<BoundaryConditionNuemann2d>();
            for (int i = 0; i < count; i++)
                linSol.boundaryConditions.Add(Reader(reader, name + "bcs" + i.ToString(), new BoundaryConditionNuemann2d()));

            linSol.peridelta = reader.GetDouble(name + "peridelta");
            linSol.elasticModulus = reader.GetDouble(name + "elasticModulus");
            linSol.bondStiffness = reader.GetDouble(name + "bondStiffness");
            linSol.oldcuttoff = reader.GetDouble(name + "oldcuttoff");

            linSol.relaxTension = reader.GetBoolean(name + "relaxTension");

            var arr = reader.GetByteArray(name + "nList");
            linSol.nList = (int[])GetObject(arr);

            return linSol;
        }
        public static Voxels2d<T> Reader<T>(GH_IReader reader, string name, Voxels2d<T> voxels)
        {
            T[] cells;
            try
            {
                var cellsO = reader.GetByteArray(name + "cells");
                if (cellsO is null)
                    return voxels;
                cells = (T[])GetObject(cellsO);
            }
            catch
            {
                return null;
            }
            int n = reader.GetInt32(name + "n");
            double delta = reader.GetDouble(name + "delta");
            var pt = reader.GetPoint2D(name + "origin");
            Point2d origin = new Point2d(pt.x, pt.y);

            voxels.cellValues = cells;
            voxels.n = n;
            voxels.origin = origin;
            voxels.delta = delta;

            return voxels;
        }
        public static BoundaryConditionNuemann2d Reader(GH_IReader reader, string name, BoundaryConditionNuemann2d bc)
        {
            bc.curve = (Curve)Curve.FromJSON(reader.GetString(name + "curve"));
            bc.normal = reader.GetBoolean(name + "normal");
            var arr = reader.GetDoubleArray(name + "load");
            bc.load = new Vector3d(arr[0], arr[1], arr[2]);

            return bc;
        }

        private static object GetObject(byte[] arrBytes)
        {
            BinaryFormatter binForm = new BinaryFormatter();
            using (MemoryStream memStream = new MemoryStream())
            {
                memStream.Write(arrBytes, 0, arrBytes.Length);
                memStream.Seek(0, SeekOrigin.Begin);
                Object obj = (Object)binForm.Deserialize(memStream);

                return obj;
            }
        }
    }
}
