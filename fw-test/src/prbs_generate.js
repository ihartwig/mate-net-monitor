// modified from https://blog.kurttomlinson.com/posts/prbs-pseudo-random-binary-sequence
var polynomial = "10100000000";
//"110000000000000" = x^15+x^14+1
//"10100000000" = x^11+x^9+1
//"1100000" = x^7+x^6+1
//"10100" = x^5+x^3+1
//"110" = x^3+x^2+1
// more https://en.wikipedia.org/wiki/Linear-feedback_shift_register#Some_polynomials_for_maximal_LFSRs
var start_state = 0x1;  /* Any nonzero start state will work. */
var taps = parseInt(polynomial, 2);
var lfsr = start_state;
var period = 0;
var prbs = "";

// generate prbs string
do
{
  var lsb = lfsr & 1;  /* Get LSB (i.e., the output bit). */
  prbs = prbs + lsb;
  lfsr >>= 1;          /* Shift register */
  if (lsb == 1) {      /* Only apply toggle mask if output bit is 1. */
    lfsr ^= taps;      /* Apply toggle mask, value has 1 at bits corresponding to taps, 0 elsewhere. */
  }
  ++period;
} while (lfsr != start_state);
console.log("period = " + period);
console.log("prbs = " + prbs);

if (period == Math.pow(2, polynomial.length)-1) {
  console.log("polynomial is maximal length");
} else {
  console.log("polynomial is not maximal length");
}

// also convert to hex
var prbs_rem = prbs;
var prbs_hex = "";
var prbs_hex_len = 0;
var prbs_pad = (Math.ceil(prbs_rem.length/8)*8)-prbs_rem.length;
prbs_rem = prbs_rem + ("0"*prbs_pad);
do {
  slice_str = prbs_rem.slice(0,8);
  slice_hex = parseInt(slice_str, 2).toString(16).padStart(2, "0");
  prbs_hex = prbs_hex + "\\x" + slice_hex;
  prbs_hex_len = prbs_hex_len + 1;
  prbs_rem = prbs_rem.slice(8);
} while (prbs_rem.length!=0);
console.log("prbs_hex_len = " + prbs_hex_len);
console.log("prbs_hex = " + prbs_hex);
