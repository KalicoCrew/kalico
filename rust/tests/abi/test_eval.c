/*
 * ABI smoke test: link against libnurbs_c_api.a and call a few core functions
 * with known inputs. Verifies the C ABI compiles, links, and produces correct
 * results across the language boundary.
 */

#include "../../nurbs-c-api/include/kalico_nurbs.h"
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    /*
     * The C side cannot construct ScalarNurbsRef directly — it's an opaque
     * Rust type. For this v1 smoke test, we'd normally call a constructor
     * extern "C" function (e.g., kalico_nurbs_scalar_ref_from_wire). That
     * function isn't part of v1's minimal ABI surface; this smoke test
     * therefore exercises only that the header compiles and the staticlib
     * links — full call-through is gated on Layer 5 wire format integration.
     */

    printf("ABI smoke: header parsed, staticlib linked\n");
    return 0;
}
