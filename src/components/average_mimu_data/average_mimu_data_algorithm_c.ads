pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings     (Off, "-gnatwu");

with Interfaces.C;         use Interfaces; use Interfaces.C;
with Acc_Data.C;
with Imu_Sensor_Body.C;
with Packed_F32x9_Record.C;

package Average_Mimu_Data_Algorithm_C is

   --* Opaque handle for an AverageMimuDataAlgorithm instance.
   type Average_Mimu_Data_Algorithm is limited private;
   type Average_Mimu_Data_Algorithm_Access is access all Average_Mimu_Data_Algorithm;

   --* @brief Construct a new AverageMimuDataAlgorithm.
   function Create
     return Average_Mimu_Data_Algorithm_Access
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_create";

   --* @brief Destroy an AverageMimuDataAlgorithm.
   procedure Destroy
     (Self : Average_Mimu_Data_Algorithm_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_destroy";

   --* @brief Run the update step to compute averaged MIMU data.
   --* @param Self          The algorithm instance.
   --* @param Acc_Data_In   Pointer to input accelerometer data message payload.
   --* @return Computed IMU sensor body output message.
   function Update
     (Self        : Average_Mimu_Data_Algorithm_Access;
      Acc_Data_In : Acc_Data.C.U_C_Access)
     return Imu_Sensor_Body.C.U_C
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_update";

   --* @brief Set the allowable time delta for averaging window.
   --* @param Self       The algorithm instance.
   --* @param Time_Delta Time delta in seconds.
   procedure Set_Time_Delta
     (Self       : Average_Mimu_Data_Algorithm_Access;
      Time_Delta : Short_Float)
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_setTimeDelta";

   --* @brief Get the current time delta for averaging window.
   --* @param Self The algorithm instance.
   --* @return The current time delta in seconds.
   function Get_Time_Delta
     (Self : Average_Mimu_Data_Algorithm_Access)
     return Short_Float
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_getTimeDelta";

   --* @brief Set the DCM from platform frame to body frame.
   --* @param Self   The algorithm instance.
   --* @param Dcm_Bp 3x3 rotation matrix in row-major POD format.
   procedure Set_Dcm_Pltf_To_Bdy
     (Self   : Average_Mimu_Data_Algorithm_Access;
      Dcm_Bp : Packed_F32x9_Record.C.U_C)
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_setDcmPltfToBdy";

   --* @brief Get the current DCM from platform frame to body frame.
   --* @param Self The algorithm instance.
   --* @return 3x3 rotation matrix in row-major POD format.
   function Get_Dcm_Pltf_To_Bdy
     (Self : Average_Mimu_Data_Algorithm_Access)
     return Packed_F32x9_Record.C.U_C
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_getDcmPltfToBdy";

private

   -- Private representation: opaque null record
   type Average_Mimu_Data_Algorithm is null record;

end Average_Mimu_Data_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings     (On, "-gnatwu");
