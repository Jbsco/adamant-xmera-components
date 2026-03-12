--------------------------------------------------------------------------------
-- Average_Mimu_Data Component Implementation Body
--------------------------------------------------------------------------------

with Ada.Numerics;
with Mimu_Data_Field_Sample_10;
with Imu_Sensor_Body;
with Packed_F32x9.C;

package body Component.Average_Mimu_Data.Implementation is

   -- ICD conversion factors for raw I32 to physical units:
   Gyro_Scale  : constant Short_Float :=
      (4_000.0 / 2_147_483_647.0) * (Ada.Numerics.Pi / 180.0);
   Accel_Scale : constant Short_Float := 160.0 / 2_147_483_647.0;

   -- Inter-sample period in nanoseconds (10 ms):
   Sample_Period_Ns : constant Interfaces.Unsigned_64 := 10_000_000;

   -- Number of raw samples per packet:
   Num_Samples : constant := 10;

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
   -- Receive raw MIMU data packet for averaging and body-frame transform.
   overriding procedure Mimu_Raw_Packet_T_Recv_Sync (Self : in out Instance; Arg : in Mimu_Raw_Packet.T) is
   begin
      Self.Update_Parameters;

      declare
         -- Convert Sys_Time to nanoseconds for the C algorithm.
         -- Sys_Time subseconds field: 1 tick = 1/65536 s.
         Base_Time_Ns : constant Interfaces.Unsigned_64 :=
            Interfaces.Unsigned_64 (Arg.Timestamp.Seconds) * 1_000_000_000 +
            Interfaces.Unsigned_64 (Arg.Timestamp.Subseconds) * 1_000_000_000 / 65_536;

         -- Unpack samples to access individual I32 fields:
         Samples : constant Mimu_Data_Field_Sample_10.U :=
            Mimu_Data_Field_Sample_10.Unpack (Arg.Samples);

         -- Build 120-element InputPktsData_c. Zero-initialized so unused
         -- slots have measTime=0, which the time-window filter excludes.
         Input : aliased Input_Pkts_Data_C := (
            Meas_Time => [others => 0],
            Gyro_P    => [others => [others => 0.0]],
            Accel_P   => [others => [others => 0.0]]
         );
      begin
         -- Convert 10 raw I32 samples to float and pack into buffer:
         for I in 0 .. Num_Samples - 1 loop
            Input.Meas_Time (I) := Base_Time_Ns + Interfaces.Unsigned_64 (I) * Sample_Period_Ns;
            Input.Gyro_P (I) := [
               Short_Float (Samples (I).Merged_Gyro_Rates.X_Measurement) * Gyro_Scale,
               Short_Float (Samples (I).Merged_Gyro_Rates.Y_Measurement) * Gyro_Scale,
               Short_Float (Samples (I).Merged_Gyro_Rates.Z_Measurement) * Gyro_Scale
            ];
            Input.Accel_P (I) := [
               Short_Float (Samples (I).Merged_Accelerations.X_Measurement) * Accel_Scale,
               Short_Float (Samples (I).Merged_Accelerations.Y_Measurement) * Accel_Scale,
               Short_Float (Samples (I).Merged_Accelerations.Z_Measurement) * Accel_Scale
            ];
         end loop;

         declare
            Output : constant Output_Average_Accel_Angle_Vel_C :=
               Update (Self.Alg, Input'Unchecked_Access);
         begin
            Self.Data_Product_T_Send (Self.Data_Products.Imu_Body_Data (
               Self.Sys_Time_T_Get,
               Imu_Sensor_Body.Pack ((
                  Dv_Frame_Body => [0.0, 0.0, 0.0],
                  Accel_Body    => [Output.Accel_B (0),
                                    Output.Accel_B (1),
                                    Output.Accel_B (2)],
                  Dr_Frame_Body => [0.0, 0.0, 0.0],
                  Ang_Vel_Body  => [Output.Gyro_Omega_B (0),
                                    Output.Gyro_Omega_B (1),
                                    Output.Gyro_Omega_B (2)]
               ))
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
      -- Set the averaging window:
      Set_Averaging_Window (Self.Alg, Self.Time_Delta.Value);
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
