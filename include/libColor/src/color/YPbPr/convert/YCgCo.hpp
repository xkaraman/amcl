#ifndef color_YPbPr_convert_YCgCo
#define color_YPbPr_convert_YCgCo

#include "../../_internal/convert.hpp"

#include "../category.hpp"


#include "../../YCgCo/YCgCo.hpp"
#include "../../rgb/rgb.hpp"

namespace color
 {
  namespace _internal
   {

    template
     <
       typename YPbPr_tag_name, ::color::constant::YPbPr::reference_enum YPbPr_reference_number
      ,typename YCgCo_tag_name
     >
     struct convert
      <
        ::color::category::YPbPr<  YPbPr_tag_name, YPbPr_reference_number >
       ,::color::category::YCgCo<  YCgCo_tag_name >
      >
      {
       public:
         typedef ::color::category::YPbPr< YPbPr_tag_name, YPbPr_reference_number >    YPbPr_category_type, category_left_type;
         typedef ::color::category::YCgCo< YCgCo_tag_name >    YCgCo_category_type, category_right_type;

         typedef typename ::color::trait::scalar<category_left_type>::instance_type  scalar_type;

         typedef ::color::model< YPbPr_category_type >  YPbPr_model_type;
         typedef ::color::model< YCgCo_category_type >  YCgCo_model_type;

         typedef ::color::rgb< scalar_type >  rgb_model_type;

         typedef ::color::trait::container<category_left_type>     container_left_trait_type;
         typedef ::color::trait::container<category_right_type>    container_right_trait_type;

         typedef typename container_left_trait_type::input_type         container_left_input_type;
         typedef typename container_right_trait_type::model_type  container_right_const_input_type;

         static void process
          (
            container_left_input_type         left
           ,container_right_const_input_type  right
          )
          {
           left = YPbPr_model_type( rgb_model_type( YCgCo_model_type( right ) ) ).container();
          }
        };

   }
 }

#endif
