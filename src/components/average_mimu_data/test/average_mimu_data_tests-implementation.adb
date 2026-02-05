--------------------------------------------------------------------------------
-- Average_Mimu_Data Tests Body
--------------------------------------------------------------------------------

with Basic_Assertions; use Basic_Assertions;
with Acc_Data;
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

      -- Uniform AccData: all 120 packets have the same gyro and accel values.
      -- With timeDelta large enough, all packets are included in the average,
      -- so the average equals the uniform value.
      Uniform_Acc_Data : constant Acc_Data.T := (
         Acc_Pkts => [others => (
            Meas_Time => 1_000_000_000, -- 1 second in nanoseconds
            Gyro_B => [1.0, 2.0, 3.0],
            Accel_B => [4.0, 5.0, 6.0]
         )]
      );

      -- Non-uniform AccData: first 60 packets zero, last 60 non-zero.
      -- Average = (60*[0,0,0] + 60*[2,4,6]) / 120 = [1,2,3] for gyro
      -- Average = (60*[0,0,0] + 60*[8,10,12]) / 120 = [4,5,6] for accel
      Mixed_Acc_Data : constant Acc_Data.T := (
         Acc_Pkts => [
            0 .. 59 => (
               Meas_Time => 1_000_000_000,
               Gyro_B => [0.0, 0.0, 0.0],
               Accel_B => [0.0, 0.0, 0.0]
            ),
            60 .. 119 => (
               Meas_Time => 1_000_000_000,
               Gyro_B => [2.0, 4.0, 6.0],
               Accel_B => [8.0, 10.0, 12.0]
            )
         ]
      );

      -- Time-filtered AccData: old packets at time=0ns, new at time=10s.
      -- With timeDelta=5.0s, only new packets are included.
      -- maxTimeTag = 10s, old: (10-0)*NANO2SEC = 10s >= 5s (excluded),
      -- new: (10-10)*NANO2SEC = 0s < 5s (included).
      Filtered_Acc_Data : constant Acc_Data.T := (
         Acc_Pkts => [
            0 .. 59 => (
               Meas_Time => 0,
               Gyro_B => [99.0, 99.0, 99.0],
               Accel_B => [99.0, 99.0, 99.0]
            ),
            60 .. 119 => (
               Meas_Time => 10_000_000_000,
               Gyro_B => [1.0, 2.0, 3.0],
               Accel_B => [4.0, 5.0, 6.0]
            )
         ]
      );
   begin
      -----------------------------------------------------------------------
      -- Set parameters: identity DCM, large time window
      -----------------------------------------------------------------------
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Time_Delta ((Value => 1.0e10))), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Dcm_Pltf_To_Bdy ([
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
         ])), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);

      -----------------------------------------------------------------------
      -- Test Case 1: Identity DCM, uniform data - output equals input
      -----------------------------------------------------------------------
      T.Acc_Data_T_Send (Uniform_Acc_Data);

      Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, 1);
      Natural_Assert.Eq (T.Imu_Body_Data_History.Get_Count, 1);

      declare
         Output : constant Imu_Sensor_Body.T := T.Imu_Body_Data_History.Get (1);
      begin
         Packed_F32x3_Assert.Eq (Output.Ang_Vel_Body, [1.0, 2.0, 3.0], Epsilon => 0.0001);
         Packed_F32x3_Assert.Eq (Output.Accel_Body, [4.0, 5.0, 6.0], Epsilon => 0.0001);
      end;

      -----------------------------------------------------------------------
      -- Test Case 2: 90-degree Z rotation DCM
      -- DCM = [0, -1, 0; 1, 0, 0; 0, 0, 1]
      -- DCM * [1,2,3] = [-2, 1, 3]
      -- DCM * [4,5,6] = [-5, 4, 6]
      -----------------------------------------------------------------------
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Dcm_Pltf_To_Bdy ([
            0.0, -1.0, 0.0,
            1.0,  0.0, 0.0,
            0.0,  0.0, 1.0
         ])), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);

      T.Acc_Data_T_Send (Uniform_Acc_Data);

      Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, 2);
      Natural_Assert.Eq (T.Imu_Body_Data_History.Get_Count, 2);

      declare
         Output : constant Imu_Sensor_Body.T := T.Imu_Body_Data_History.Get (2);
      begin
         Packed_F32x3_Assert.Eq (Output.Ang_Vel_Body, [-2.0, 1.0, 3.0], Epsilon => 0.0001);
         Packed_F32x3_Assert.Eq (Output.Accel_Body, [-5.0, 4.0, 6.0], Epsilon => 0.0001);
      end;

      -----------------------------------------------------------------------
      -- Test Case 3: Non-uniform data, identity DCM - tests averaging
      -- First 60 pkts: gyro=[0,0,0], accel=[0,0,0]
      -- Last 60 pkts: gyro=[2,4,6], accel=[8,10,12]
      -- Average: gyro=[1,2,3], accel=[4,5,6]
      -- Like Python test: validates averaging over varied packet data.
      -----------------------------------------------------------------------
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Dcm_Pltf_To_Bdy ([
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
         ])), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);

      T.Acc_Data_T_Send (Mixed_Acc_Data);

      Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, 3);
      Natural_Assert.Eq (T.Imu_Body_Data_History.Get_Count, 3);

      declare
         Output : constant Imu_Sensor_Body.T := T.Imu_Body_Data_History.Get (3);
      begin
         Packed_F32x3_Assert.Eq (Output.Ang_Vel_Body, [1.0, 2.0, 3.0], Epsilon => 0.0001);
         Packed_F32x3_Assert.Eq (Output.Accel_Body, [4.0, 5.0, 6.0], Epsilon => 0.0001);
      end;

      -----------------------------------------------------------------------
      -- Test Case 4: Time filtering - small timeDelta excludes old packets
      -- Old 60 pkts at time=0ns with bogus values [99,99,99]
      -- New 60 pkts at time=10s with gyro=[1,2,3], accel=[4,5,6]
      -- timeDelta=5.0s: maxTimeTag=10s
      --   old: (10-0)*NANO2SEC = 10s >= 5s -> excluded
      --   new: (10-10)*NANO2SEC = 0s < 5s -> included
      -- Average of new packets only: gyro=[1,2,3], accel=[4,5,6]
      -----------------------------------------------------------------------
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Time_Delta ((Value => 5.0))), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);

      T.Acc_Data_T_Send (Filtered_Acc_Data);

      Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, 4);
      Natural_Assert.Eq (T.Imu_Body_Data_History.Get_Count, 4);

      declare
         Output : constant Imu_Sensor_Body.T := T.Imu_Body_Data_History.Get (4);
      begin
         Packed_F32x3_Assert.Eq (Output.Ang_Vel_Body, [1.0, 2.0, 3.0], Epsilon => 0.0001);
         Packed_F32x3_Assert.Eq (Output.Accel_Body, [4.0, 5.0, 6.0], Epsilon => 0.0001);
      end;
   end Test;

end Average_Mimu_Data_Tests.Implementation;
