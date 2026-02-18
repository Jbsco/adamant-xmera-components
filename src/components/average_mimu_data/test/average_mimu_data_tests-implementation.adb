--------------------------------------------------------------------------------
-- Average_Mimu_Data Tests Body
--------------------------------------------------------------------------------

with Ada.Numerics;
with Interfaces; use Interfaces;
with Basic_Assertions; use Basic_Assertions;
with Mimu_Raw_Packet;
with Imu_Sensor_Body;
with Packed_F32x3.Assertion; use Packed_F32x3.Assertion;
with Average_Mimu_Data_Parameters;
with Parameter_Enums.Assertion;
use Parameter_Enums.Parameter_Update_Status;
use Parameter_Enums.Assertion;

package body Average_Mimu_Data_Tests.Implementation is

   -------------------------------------------------------------------------
   -- Fixtures:
   -------------------------------------------------------------------------

   overriding procedure Set_Up_Test (Self : in out Instance) is
   begin
      -- Allocate heap memory to component:
      Self.Tester.Init_Base;

      -- Make necessary connections between tester and component:
      Self.Tester.Connect;

      -- Call component init here.
      Self.Tester.Component_Instance.Init;

      -- Call the component set up method that the assembly would normally call.
      Self.Tester.Component_Instance.Set_Up;
   end Set_Up_Test;

   overriding procedure Tear_Down_Test (Self : in out Instance) is
   begin
      -- Free component heap:
      Self.Tester.Component_Instance.Destroy;
      Self.Tester.Final_Base;
   end Tear_Down_Test;

   -------------------------------------------------------------------------
   -- Tests:
   -------------------------------------------------------------------------

   -- Run algorithm to ensure integration is sound.
   overriding procedure Test (Self : in out Instance) is
      T : Component.Average_Mimu_Data.Implementation.Tester.Instance_Access renames Self.Tester;
      Params : Average_Mimu_Data_Parameters.Instance;

      -- ICD conversion factors (must match component implementation):
      -- gyro[deg/s] = dn * 4000/2147483647, then deg->rad
      -- acc[m/s^2]  = dn * 160/2147483647
      Gyro_Scale : constant Short_Float :=
         (4_000.0 / 2_147_483_647.0) * (Ada.Numerics.Pi / 180.0);
      Accel_Scale : constant Short_Float := 160.0 / 2_147_483_647.0;

      -- Expected physical-unit values for raw dn = 1M .. 6M:
      G1 : constant Short_Float := 1_000_000.0 * Gyro_Scale;
      G2 : constant Short_Float := 2_000_000.0 * Gyro_Scale;
      G3 : constant Short_Float := 3_000_000.0 * Gyro_Scale;
      A4 : constant Short_Float := 4_000_000.0 * Accel_Scale;
      A5 : constant Short_Float := 5_000_000.0 * Accel_Scale;
      A6 : constant Short_Float := 6_000_000.0 * Accel_Scale;

      -- Uniform raw packet: all 10 samples have the same integer values.
      -- ICD scale: 1_000_000 -> G1, 2_000_000 -> G2, etc.
      -- Timestamp Seconds=1 so the 10 filled samples have measTime ~1s.
      -- The 110 zero-filled Acc_Data slots (measTime=0) have age ~1.09s,
      -- which is excluded by timeDelta=1.0.
      Uniform_Raw_Packet : constant Mimu_Raw_Packet.T := (
         Timestamp => (Seconds => 1, Subseconds => 0),
         Samples => [others => (
            Merged_Gyro_Rates => (X_Measurement => 1_000_000, Y_Measurement => 2_000_000, Z_Measurement => 3_000_000),
            Merged_Accelerations => (X_Measurement => 4_000_000, Y_Measurement => 5_000_000, Z_Measurement => 6_000_000),
            Merge_Info => 0
         )]
      );

      -- Non-uniform raw packet with negative values: first 5 samples negative,
      -- last 5 positive. Tests signed Integer_32-to-float conversion and
      -- averaging across mixed signs.
      -- After ICD scale:
      --   first 5 gyro = [-G1, -G2, -G3], accel = [-A4, -A5, -A6]
      --   last 5 gyro = [3*G1, 3*G2, 3*G3], accel = [3*A4, 3*A5, 3*A6]
      -- Average of 10: gyro = [G1, G2, G3], accel = [A4, A5, A6]
      Mixed_Raw_Packet : constant Mimu_Raw_Packet.T := (
         Timestamp => (Seconds => 1, Subseconds => 0),
         Samples => [
            0 .. 4 => (
               Merged_Gyro_Rates => (X_Measurement => -1_000_000, Y_Measurement => -2_000_000, Z_Measurement => -3_000_000),
               Merged_Accelerations => (X_Measurement => -4_000_000, Y_Measurement => -5_000_000, Z_Measurement => -6_000_000),
               Merge_Info => 0
            ),
            5 .. 9 => (
               Merged_Gyro_Rates => (X_Measurement => 3_000_000, Y_Measurement => 6_000_000, Z_Measurement => 9_000_000),
               Merged_Accelerations => (X_Measurement => 12_000_000, Y_Measurement => 15_000_000, Z_Measurement => 18_000_000),
               Merge_Info => 0
            )
         ]
      );

      -- Time-filtered raw packet: bogus values in samples 0-4, known values in 5-9.
      -- With timeDelta=0.045 (45ms), per-sample timestamps are base + I*10ms:
      --   maxTimeTag = base + 90ms
      --   Sample I age = (9-I)*10ms; included when age*NANO2SEC < timeDelta.
      --   Sample 4: age=50ms, 0.05 < 0.045 => NO (excluded)
      --   Sample 5: age=40ms, 0.04 < 0.045 => YES (included)
      --   Only samples 5-9 pass the time filter.
      -- Average of samples 5-9: gyro=[G1,G2,G3], accel=[A4,A5,A6]
      Filtered_Raw_Packet : constant Mimu_Raw_Packet.T := (
         Timestamp => (Seconds => 1, Subseconds => 0),
         Samples => [
            0 .. 4 => (
               Merged_Gyro_Rates => (X_Measurement => 99_000_000, Y_Measurement => 99_000_000, Z_Measurement => 99_000_000),
               Merged_Accelerations => (X_Measurement => 99_000_000, Y_Measurement => 99_000_000, Z_Measurement => 99_000_000),
               Merge_Info => 0
            ),
            5 .. 9 => (
               Merged_Gyro_Rates => (X_Measurement => 1_000_000, Y_Measurement => 2_000_000, Z_Measurement => 3_000_000),
               Merged_Accelerations => (X_Measurement => 4_000_000, Y_Measurement => 5_000_000, Z_Measurement => 6_000_000),
               Merge_Info => 0
            )
         ]
      );
   begin
      -----------------------------------------------------------------------
      -- Set parameters: identity DCM, 1s time window
      -- timeDelta=1.0 includes all 10 filled samples (age 0-90ms < 1.0s)
      -- but excludes the 110 zero-filled slots (age ~1.09s >= 1.0s).
      -----------------------------------------------------------------------
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Time_Delta ((Value => 1.0))), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Dcm_Pltf_To_Bdy ([
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
         ])), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);

      -----------------------------------------------------------------------
      -- Test Case 1: Identity DCM, uniform data - output equals scaled input
      -- All 10 samples: Integer [1M,2M,3M] -> Float [G1,G2,G3] gyro
      -----------------------------------------------------------------------
      T.Mimu_Raw_Packet_T_Send (Uniform_Raw_Packet);

      Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, 1);
      Natural_Assert.Eq (T.Imu_Body_Data_History.Get_Count, 1);

      declare
         Output : constant Imu_Sensor_Body.T := T.Imu_Body_Data_History.Get (1);
      begin
         Packed_F32x3_Assert.Eq (Output.Ang_Vel_Body, [G1, G2, G3], Epsilon => 0.0001);
         Packed_F32x3_Assert.Eq (Output.Accel_Body, [A4, A5, A6], Epsilon => 0.0001);
      end;

      -----------------------------------------------------------------------
      -- Test Case 2: 90-degree Z rotation DCM
      -- DCM = [0, -1, 0; 1, 0, 0; 0, 0, 1]
      -- DCM * [G1,G2,G3] = [-G2, G1, G3]
      -- DCM * [A4,A5,A6] = [-A5, A4, A6]
      -----------------------------------------------------------------------
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Dcm_Pltf_To_Bdy ([
            0.0, -1.0, 0.0,
            1.0,  0.0, 0.0,
            0.0,  0.0, 1.0
         ])), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);

      T.Mimu_Raw_Packet_T_Send (Uniform_Raw_Packet);

      Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, 2);
      Natural_Assert.Eq (T.Imu_Body_Data_History.Get_Count, 2);

      declare
         Output : constant Imu_Sensor_Body.T := T.Imu_Body_Data_History.Get (2);
      begin
         Packed_F32x3_Assert.Eq (Output.Ang_Vel_Body, [-G2, G1, G3], Epsilon => 0.0001);
         Packed_F32x3_Assert.Eq (Output.Accel_Body, [-A5, A4, A6], Epsilon => 0.0001);
      end;

      -----------------------------------------------------------------------
      -- Test Case 3: Non-uniform data with negative values, identity DCM
      -- Tests signed I32-to-float conversion and averaging across mixed signs.
      -- First 5: gyro=[-1M,-2M,-3M] -> [-G1,-G2,-G3]
      -- Last 5: gyro=[3M,6M,9M] -> [3*G1,3*G2,3*G3]
      -- Average of 10: gyro=[G1,G2,G3], accel=[A4,A5,A6]
      -----------------------------------------------------------------------
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Dcm_Pltf_To_Bdy ([
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
         ])), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);

      T.Mimu_Raw_Packet_T_Send (Mixed_Raw_Packet);

      Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, 3);
      Natural_Assert.Eq (T.Imu_Body_Data_History.Get_Count, 3);

      declare
         Output : constant Imu_Sensor_Body.T := T.Imu_Body_Data_History.Get (3);
      begin
         Packed_F32x3_Assert.Eq (Output.Ang_Vel_Body, [G1, G2, G3], Epsilon => 0.0001);
         Packed_F32x3_Assert.Eq (Output.Accel_Body, [A4, A5, A6], Epsilon => 0.0001);
      end;

      -----------------------------------------------------------------------
      -- Test Case 4: Time filtering within 10-sample packet
      -- Samples 0-4: bogus values [99M,99M,99M]
      -- Samples 5-9: gyro=[1M,2M,3M], accel=[4M,5M,6M]
      -- timeDelta=0.045 (45ms), per-sample timestamps: base + I*10ms
      --   maxTimeTag = base + 90ms
      --   Sample I included when (9-I)*10ms * NANO2SEC < 0.045
      --   Samples 0-4 excluded (ages 50-90ms), samples 5-9 included (ages 0-40ms)
      -- Average of samples 5-9 only: gyro=[G1,G2,G3], accel=[A4,A5,A6]
      -- Note: timeDelta=0.045 avoids exact boundary at 50ms where IEEE 754
      -- float/double promotion in the C++ filter would include sample 4.
      -----------------------------------------------------------------------
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Time_Delta ((Value => 0.045))), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);

      T.Mimu_Raw_Packet_T_Send (Filtered_Raw_Packet);

      Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, 4);
      Natural_Assert.Eq (T.Imu_Body_Data_History.Get_Count, 4);

      declare
         Output : constant Imu_Sensor_Body.T := T.Imu_Body_Data_History.Get (4);
      begin
         Packed_F32x3_Assert.Eq (Output.Ang_Vel_Body, [G1, G2, G3], Epsilon => 0.0001);
         Packed_F32x3_Assert.Eq (Output.Accel_Body, [A4, A5, A6], Epsilon => 0.0001);
      end;

      -----------------------------------------------------------------------
      -- Test Case 5: Sequential invocations - verify no state leaks
      -- The C++ algorithm's update() is const; each call should be
      -- independent. Send the same uniform packet twice with identity DCM
      -- and timeDelta=1.0 and confirm identical output each time.
      -- Also exercises parameter change back from timeDelta=0.045 to 1.0.
      -----------------------------------------------------------------------
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Time_Delta ((Value => 1.0))), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Dcm_Pltf_To_Bdy ([
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
         ])), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);

      T.Mimu_Raw_Packet_T_Send (Uniform_Raw_Packet);
      T.Mimu_Raw_Packet_T_Send (Uniform_Raw_Packet);

      Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, 6);
      Natural_Assert.Eq (T.Imu_Body_Data_History.Get_Count, 6);

      declare
         Output_5 : constant Imu_Sensor_Body.T := T.Imu_Body_Data_History.Get (5);
         Output_6 : constant Imu_Sensor_Body.T := T.Imu_Body_Data_History.Get (6);
      begin
         Packed_F32x3_Assert.Eq (Output_5.Ang_Vel_Body, [G1, G2, G3], Epsilon => 0.0001);
         Packed_F32x3_Assert.Eq (Output_5.Accel_Body, [A4, A5, A6], Epsilon => 0.0001);
         Packed_F32x3_Assert.Eq (Output_6.Ang_Vel_Body, [G1, G2, G3], Epsilon => 0.0001);
         Packed_F32x3_Assert.Eq (Output_6.Accel_Body, [A4, A5, A6], Epsilon => 0.0001);
      end;
   end Test;

end Average_Mimu_Data_Tests.Implementation;
