#ifndef VCD_DEFINITIONS_H_INCLUDED
#define VCD_DEFINITIONS_H_INCLUDED

#define VCD_VAR_SYMBOLE         "$var"
#define VCD_TIMESCALE_SYMBOLE   "$timescale"
#define VCD_END_SYMBOLE         "$end"
#define VCD_DATE_SYMBOLE        "$date"
#define VCD_VERSION_SYMBOLE     "$version"
#define VCD_COMMENT_SYMBOLE     "$comment"
#define VCD_SCOPE_SYMBOLE       "$scope"
#define VCD_ENDDEFINITION_SYMBOLE "$enddefinitions"
#define VCD_DUMPVARS_SYMBOLE     "$dumpvars"

#define VCD_TIMESTAMP_SYMBOLE   '#'

#define VCD_NUMBER_BASE         10

#define VCD_ps                  "ps"
#define VCD_ns                  "ns"
#define VCD_us                  "us"
#define VCD_ms                  "ms"
#define VCD_s                   "s"

#define VCD_ps_value            (double)(1.0/1000000000000)
#define VCD_ns_value            (double)(1.0/1000000000)
#define VCD_us_value            (double)(1.0/1000000)
#define VCD_ms_value            (double)(1.0/1000)
#define VCD_s_value             (double)(1.0)

#define VCD_VALUE_HIGH          '1'
#define VCD_VALUE_LOW           '0'
#define VCD_VALUE_UNKNOW        'x'
#define VCD_VALUE_HIGHIMPEDANCE 'z'
#define VCD_VALUE_BINARY        'b'
#define VCD_VALUE_DECIMAL       'd'
#define VCD_VALUE_HEXADECIMAL   'h'

#define VCD_VALUE_HIGH_BIN      (int)1
#define VCD_VALUE_LOW_BIN       (int)0
#define VCD_VALUE_UNKNOW_BIN    (int)0
#define VCD_VALUE_HIGHIMPEDANCE_BIN (int)0


#endif // VCD_DEFINITIONS_H_INCLUDED
