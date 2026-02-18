--------------------------------------------------------------------------------
-- Average_Mimu_Data Component Implementation Body
--------------------------------------------------------------------------------

with Ada.Numerics;
with Acc_Data;
with Acc_Data.C;
with Imu_Sensor_Body.C;
with Packed_F32x9.C;

package body Component.Average_Mimu_Data.Implementation is

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the AverageMimuData algorithm.
   overriding procedure Init (Self : in out Instance) is
   begin
      -- Allocate C++ class on the heap
      Self.Alg := Create;
   end Init;

   not overriding procedure Destroy (Self : in out Instance) is
   begin
      -- Free the C++ heap data.
      Destroy (Self.Alg);
   end Destroy;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- Receive raw MIMU data packet for conversion, averaging, and body-frame
   -- transform.
   overriding procedure Mimu_Raw_Packet_T_Recv_Sync (Self : in out Instance; Arg : in Mimu_Raw_Packet.T) is
   begin
      -- Update the parameters:
      Self.Update_Parameters;

      declare
         -- Access fields directly on the packed .T argument. No Unpack needed
         -- since packed records support field access via representation clauses.

         -- Build the 120-slot Acc_Data buffer expected by the C++ algorithm.
         -- Only the first 10 slots are filled from the MIMU packet; the
         -- remaining 110 stay zero-initialized. The C++ algorithm's time-window
         -- filtering excludes stale/zero entries automatically.
         Acc_Buffer : Acc_Data.U := (Acc_Pkts => [others => (
            Meas_Time => 0,
            Gyro_B => [0.0, 0.0, 0.0],
            Accel_B => [0.0, 0.0, 0.0]
         )]);

         -- The MIMU provides raw measurements as Integer_32 in DPU-internal units.
         -- The C++ averageMimuData algorithm expects physical units: gyro in [r/s]
         -- and accel in [m/s^2] as IEEE 754 floats. These scale factors convert
         -- from the DPU integer representation to physical units.
         -- ICD: gyro[deg/s] = dn * 4000/2147483647, then deg->rad
         -- ICD: acc[m/s^2]  = dn * 160/2147483647
         Gyro_Scale_Factor : constant Short_Float :=
            (4_000.0 / 2_147_483_647.0) * (Ada.Numerics.Pi / 180.0);
         Accel_Scale_Factor : constant Short_Float :=
            160.0 / 2_147_483_647.0;

         -- Synthesize a base timestamp in nanoseconds from the packet's Sys_Time.
         -- Sys_Time subseconds field: 1 tick = 1/65536 s.
         -- Convert to nanoseconds: subseconds * 1e9 / 65536.
         Base_Time_Ns : constant Interfaces.Unsigned_64 :=
            Interfaces.Unsigned_64 (Arg.Timestamp.Seconds) * 1_000_000_000 +
            Interfaces.Unsigned_64 (Arg.Timestamp.Subseconds) * 1_000_000_000 / 65_536;
      begin
         -- Convert 10 MIMU samples to Acc_Pkt_Data format.
         for I in 0 .. 9 loop
            -- Per-sample timestamp: base time + 10ms offset (100 Hz sample rate)
            Acc_Buffer.Acc_Pkts (I).Meas_Time :=
               Base_Time_Ns + Interfaces.Unsigned_64 (I) * 10_000_000;

            -- Scale Integer_32 measurements to Short_Float in physical units.
            Acc_Buffer.Acc_Pkts (I).Gyro_B := [
               Short_Float (Arg.Samples (I).Merged_Gyro_Rates.X_Measurement) * Gyro_Scale_Factor,
               Short_Float (Arg.Samples (I).Merged_Gyro_Rates.Y_Measurement) * Gyro_Scale_Factor,
               Short_Float (Arg.Samples (I).Merged_Gyro_Rates.Z_Measurement) * Gyro_Scale_Factor
            ];

            Acc_Buffer.Acc_Pkts (I).Accel_B := [
               Short_Float (Arg.Samples (I).Merged_Accelerations.X_Measurement) * Accel_Scale_Factor,
               Short_Float (Arg.Samples (I).Merged_Accelerations.Y_Measurement) * Accel_Scale_Factor,
               Short_Float (Arg.Samples (I).Merged_Accelerations.Z_Measurement) * Accel_Scale_Factor
            ];
         end loop;

         -- Convert Ada types to C types and call the algorithm:
         declare
            Acc_Data_C : aliased Acc_Data.C.U_C := Acc_Data.C.To_C (Acc_Buffer);

            -- Call the C algorithm:
            Imu_Output : constant Imu_Sensor_Body.C.U_C := Update (
               Self.Alg,
               Acc_Data_In => Acc_Data_C'Unchecked_Access
            );
         begin
            -- Send out data product:
            Self.Data_Product_T_Send (Self.Data_Products.Imu_Body_Data (
               Self.Sys_Time_T_Get,
               Imu_Sensor_Body.Pack (Imu_Sensor_Body.C.To_Ada (Imu_Output))
            ));
         end;
      end;
   end Mimu_Raw_Packet_T_Recv_Sync;

   -- The parameter update connector.
   overriding procedure Parameter_Update_T_Modify (Self : in out Instance; Arg : in out Parameter_Update.T) is
   begin
      -- Process the parameter update, staging or fetching parameters as requested.
      Self.Process_Parameter_Update (Arg);
   end Parameter_Update_T_Modify;

   -----------------------------------------------
   -- Parameter handlers:
   -----------------------------------------------
   -- Description:
   --    Parameters for the Average Mimu Data component
   -- Apply parameter values to the C++ algorithm when parameters change.
   overriding procedure Update_Parameters_Action (Self : in out Instance) is
   begin
      -- Set the averaging window time delta:
      Set_Time_Delta (Self.Alg, Self.Time_Delta.Value);
      -- Set the platform-to-body DCM:
      Set_Dcm_Pltf_To_Bdy (Self.Alg, (Value => Packed_F32x9.C.To_C (Self.Dcm_Pltf_To_Bdy)));
   end Update_Parameters_Action;

   -- Invalid Parameter handler. This procedure is called when a parameter's type is found to be invalid:
   overriding procedure Invalid_Parameter (Self : in out Instance; Par : in Parameter.T; Errant_Field_Number : in Unsigned_32; Errant_Field : in Basic_Types.Poly_Type) is
      pragma Annotate (GNATSAS, Intentional, "subp always fails", "intentional assertion");
   begin
      -- None of the parameters should be invalid in this case.
      pragma Assert (False);
   end Invalid_Parameter;

end Component.Average_Mimu_Data.Implementation;
