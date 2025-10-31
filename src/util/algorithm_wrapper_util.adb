--------------------------------------------------------------------------------
-- Algorithm Wrapper Utility Package Body
--------------------------------------------------------------------------------

package body Algorithm_Wrapper_Util is

   function Is_Dep_Status_Success (Status : in Data_Product_Enums.Data_Dependency_Status.E) return Boolean is
      use Data_Product_Enums.Data_Dependency_Status;
   begin
      case Status is
         when Success => return True;
         when Not_Available | Stale => return False;
         when Error => pragma Assert (False);
      end case;
   end Is_Dep_Status_Success;

end Algorithm_Wrapper_Util;
