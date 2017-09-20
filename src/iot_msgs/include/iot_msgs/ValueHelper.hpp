#include "rclcpp/rclcpp.hpp"
#include "iot_msgs/msg/instance.hpp"

namespace iot_msgs
{
  template <typename T> struct type2int {};

  template<> struct type2int<uint8_t> { enum { result = msg::NVP::TYPE_UINT8 }; };
  template<> struct type2int<uint16_t> { enum { result = msg::NVP::TYPE_UINT16 }; };
  template<> struct type2int<uint32_t> { enum { result = msg::NVP::TYPE_UINT32 }; };
  template<> struct type2int<uint64_t> { enum { result = msg::NVP::TYPE_UINT64 }; };
  template<> struct type2int<int8_t> { enum { result = msg::NVP::TYPE_INT8 }; };
  template<> struct type2int<int16_t> { enum { result = msg::NVP::TYPE_INT16 }; };
  template<> struct type2int<int32_t> { enum { result = msg::NVP::TYPE_INT32 }; };
  template<> struct type2int<int64_t> { enum { result = msg::NVP::TYPE_INT64 }; };
  template<> struct type2int<float> { enum { result = msg::NVP::TYPE_FLOAT }; };
  template<> struct type2int<double> { enum { result = msg::NVP::TYPE_DOUBLE }; };
  template<> struct type2int<bool> { enum { result = msg::NVP::TYPE_BOOL }; };
  template<> struct type2int<std::string> { enum { result = msg::NVP::TYPE_STRING }; };

  namespace ValueHelper
  {
    template<class T> std::vector<T> & getSeq (iot_msgs::msg::NVP & nvp);

    template<> std::vector<uint8_t> & getSeq (iot_msgs::msg::NVP & nvp) { return nvp.uint8_seq; }
    template<> std::vector<uint16_t> & getSeq (iot_msgs::msg::NVP & nvp) { return nvp.uint16_seq; }
    template<> std::vector<uint32_t> & getSeq (iot_msgs::msg::NVP & nvp) { return nvp.uint32_seq; }
    template<> std::vector<uint64_t> & getSeq (iot_msgs::msg::NVP & nvp) { return nvp.uint64_seq; }
    template<> std::vector<int8_t> & getSeq (iot_msgs::msg::NVP & nvp) { return nvp.int8_seq; }
    template<> std::vector<int16_t> & getSeq (iot_msgs::msg::NVP & nvp) { return nvp.int16_seq; }
    template<> std::vector<int32_t> & getSeq (iot_msgs::msg::NVP & nvp) { return nvp.int32_seq; }
    template<> std::vector<int64_t> & getSeq (iot_msgs::msg::NVP & nvp) { return nvp.int64_seq; }
    template<> std::vector<float> & getSeq (iot_msgs::msg::NVP & nvp) { return nvp.float32_seq; }
    template<> std::vector<double> & getSeq (iot_msgs::msg::NVP & nvp) { return nvp.float64_seq; }
    template<> std::vector<bool> & getSeq (iot_msgs::msg::NVP & nvp) { return nvp.bool_seq; }
    template<> std::vector<std::string> & getSeq (iot_msgs::msg::NVP & nvp) { return nvp.string_seq; }

    template<class T> void setSeq (iot_msgs::msg::NVP & nvp, std::vector<T> & seq);

    template<> void setSeq (iot_msgs::msg::NVP & nvp, std::vector<uint8_t> & seq) { nvp.set__uint8_seq (seq); }
    template<> void setSeq (iot_msgs::msg::NVP & nvp, std::vector<uint16_t> & seq) { nvp.set__uint16_seq (seq); }
    template<> void setSeq (iot_msgs::msg::NVP & nvp, std::vector<uint32_t> & seq) { nvp.set__uint32_seq (seq); }
    template<> void setSeq (iot_msgs::msg::NVP & nvp, std::vector<uint64_t> & seq) { nvp.set__uint64_seq (seq); }
    template<> void setSeq (iot_msgs::msg::NVP & nvp, std::vector<int8_t> & seq) { nvp.set__int8_seq (seq); }
    template<> void setSeq (iot_msgs::msg::NVP & nvp, std::vector<int16_t> & seq) { nvp.set__int16_seq (seq); }
    template<> void setSeq (iot_msgs::msg::NVP & nvp, std::vector<int32_t> & seq) { nvp.set__int32_seq (seq); }
    template<> void setSeq (iot_msgs::msg::NVP & nvp, std::vector<int64_t> & seq) { nvp.set__int64_seq (seq); }
    template<> void setSeq (iot_msgs::msg::NVP & nvp, std::vector<float> & seq) { nvp.set__float32_seq (seq); }
    template<> void setSeq (iot_msgs::msg::NVP & nvp, std::vector<double> & seq) { nvp.set__float64_seq (seq); }
    template<> void setSeq (iot_msgs::msg::NVP & nvp, std::vector<bool> & seq) { nvp.set__bool_seq (seq); }
    template<> void setSeq (iot_msgs::msg::NVP & nvp, std::vector<std::string> & seq) { nvp.set__string_seq (seq); }

    template<class T> void add (std::vector<iot_msgs::msg::NVP> & vals, const char * name, const T & val)
    {
      auto nvp = iot_msgs::msg::NVP ();
      std::vector<T> seq;

      nvp.set__name (name);
      seq.insert (seq.end (), val);
      setSeq<T> (nvp, seq);
      nvp.set__value_type (type2int<T>::result);
      vals.push_back (nvp);
    }

    void printValue (iot_msgs::msg::NVP & nvp)
    {
      switch (nvp.value_type)
      {
        case iot_msgs::msg::NVP::TYPE_UINT8: printf ("%u", getSeq<uint8_t> (nvp)[0]); break;
        case iot_msgs::msg::NVP::TYPE_UINT16: printf ("%u", getSeq<uint16_t> (nvp)[0]); break;
        case iot_msgs::msg::NVP::TYPE_UINT32: printf ("%u", getSeq<uint32_t> (nvp)[0]); break;
        case iot_msgs::msg::NVP::TYPE_UINT64: printf ("%lu", (long unsigned int) getSeq<uint64_t> (nvp)[0]); break;
        case iot_msgs::msg::NVP::TYPE_INT8: printf ("%d", getSeq<int8_t> (nvp)[0]); break;
        case iot_msgs::msg::NVP::TYPE_INT16: printf ("%d", getSeq<int16_t> (nvp)[0]); break;
        case iot_msgs::msg::NVP::TYPE_INT32: printf ("%d", getSeq<int32_t> (nvp)[0]); break;
        case iot_msgs::msg::NVP::TYPE_INT64: printf ("%ld", (long int) getSeq<int64_t> (nvp)[0]); break;
        case iot_msgs::msg::NVP::TYPE_FLOAT: printf ("%f", getSeq<float> (nvp)[0]); break;
        case iot_msgs::msg::NVP::TYPE_DOUBLE: printf ("%lf", getSeq<double> (nvp)[0]); break;
        case iot_msgs::msg::NVP::TYPE_BOOL: printf (getSeq<bool> (nvp)[0] ? "true" : "false"); break;
        case iot_msgs::msg::NVP::TYPE_STRING: printf ("%s", (getSeq<std::string> (nvp)[0]).c_str ()); break;
        default: printf ("?"); break;
      }
    }

    void printType (uint32_t type)
    {
      static const char * types [12] =
      {
        "uint8_t", "uint16_t", "uint32_t", "uint64_t", "int8_t", "int16_t",
        "int32_t", "int64_t", "float", "double", "bool", "string"
      };
      assert (type < 12);
      printf ("%s", types[type]);
    }

    void print (iot_msgs::msg::NVP & nvp)
    {
      printf ("  %s = ", nvp.name.c_str ());
      printValue (nvp);
      printf (" (");
      printType (nvp.value_type);
      printf (")\n");
    }
  }
}
