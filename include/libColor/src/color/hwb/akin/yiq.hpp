#ifndef color_hwb_akin_yiq
#define color_hwb_akin_yiq

#include "../../generic/akin/hwb.hpp"
#include "../category.hpp"
#include "../../yiq/category.hpp"

namespace color
 {
  namespace akin
   {

    template< >struct hwb< ::color::category::yiq_uint8   >{ typedef ::color::category::hwb_uint8   akin_type; };
    template< >struct hwb< ::color::category::yiq_uint16  >{ typedef ::color::category::hwb_uint16  akin_type; };
    template< >struct hwb< ::color::category::yiq_uint32  >{ typedef ::color::category::hwb_uint32  akin_type; };
    template< >struct hwb< ::color::category::yiq_uint64  >{ typedef ::color::category::hwb_uint64  akin_type; };
    template< >struct hwb< ::color::category::yiq_float   >{ typedef ::color::category::hwb_float   akin_type; };
    template< >struct hwb< ::color::category::yiq_double  >{ typedef ::color::category::hwb_double  akin_type; };
    template< >struct hwb< ::color::category::yiq_ldouble >{ typedef ::color::category::hwb_ldouble akin_type; };

   }
 }

#endif
