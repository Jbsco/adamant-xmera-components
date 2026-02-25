--------------------------------------------------------------------------------
-- Sunline_Srukf Component Implementation Body
--------------------------------------------------------------------------------

package body Component.Sunline_Srukf.Implementation is

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the sunline SRuKF algorithm.
   overriding procedure Init (Self : in out Instance) is
      -- TODO declarations
   begin
      null; -- TODO statements
   end Init;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- Run the algorithm up to the current time.
   overriding procedure Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T) is
      -- TODO declarations
   begin
      null; -- TODO statements
   end Tick_T_Recv_Sync;

   -----------------------------------------------
   -- Data dependency handlers:
   -----------------------------------------------
   -- Description:
   --    Data dependencies for the Sunline SRuKF component.
   -- Invalid data dependency handler. This procedure is called when a data dependency's id or length are found to be invalid:
   overriding procedure Invalid_Data_Dependency (Self : in out Instance; Id : in Data_Product_Types.Data_Product_Id; Ret : in Data_Product_Return.T) is
   begin
      -- TODO: Perform action to handle an invalid data dependency.
      -- Example:
      -- -- Throw event:
      -- Self.Event_T_Send_If_Connected (Self.Events.Invalid_Data_Dependency_Received (
      --    Self.Sys_Time_T_Get,
      --    (Id => Id, Request_Status => Ret.The_Status, Header => Ret.The_Data_Product.Header)
      -- ));
      null;
   end Invalid_Data_Dependency;

end Component.Sunline_Srukf.Implementation;
