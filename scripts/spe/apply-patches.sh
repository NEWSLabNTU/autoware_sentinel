#!/usr/bin/env bash
#
# Phase 11.5.c — apply the BSP integration edits.
#
# Idempotent text patches (not unified diffs — line numbers shift
# across L4T point releases, so we anchor on stable code patterns
# instead). Each edit is guarded by an `ENABLE_NROS_APP` marker so
# repeat runs are no-ops. Reverse direction lives in
# `apply-patches.sh --revert`.
#
# Env knobs:
#   SPE_BSP_SRC_DIR — pre-extracted SPE BSP root
#                     (default: scripts/spe/downloads/spe-freertos-bsp)
#
# Exit codes:
#   0 — all edits applied (or already applied)
#   1 — BSP not found or anchor missing

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BSP_DEFAULT="$SCRIPT_DIR/downloads/spe-freertos-bsp"
BSP="${SPE_BSP_SRC_DIR:-$BSP_DEFAULT}"

info(){ printf '\033[1;34m==> %s\033[0m\n' "$*"; }
die(){  printf '\033[1;31m==> %s\033[0m\n' "$*" >&2; exit 1; }

[ -d "$BSP/rt-aux-cpu-demo-fsp" ] || \
    die "BSP not found at $BSP. Run: just orin_spe-bsp-download"

MAKEFILE="$BSP/rt-aux-cpu-demo-fsp/Makefile"
MAINC="$BSP/rt-aux-cpu-demo-fsp/main.c"
SHIM_SRC="$SCRIPT_DIR/app/nros-app.c"
SHIM_DST="$BSP/rt-aux-cpu-demo-fsp/app/nros-app.c"

# ----------------------------------------------------------------------
# Edit 1: Makefile — add CFLAGS branch + SRCS branch for ENABLE_NROS_APP.
#
# Anchor (CFLAGS branch): the LAST `endif` ahead of `LDFLAGS := \` is
# the ENABLE_SPE_FOR_ORIN_NANO close. Inject after it.
# Anchor (SRCS branch):   the LAST `endif` of the `ifeq ($(ENABLE_SPI_SLV_APP)
# SRCS += ...)` block. Inject after it.
# ----------------------------------------------------------------------

if grep -q 'ENABLE_NROS_APP' "$MAKEFILE"; then
    info "Makefile: ENABLE_NROS_APP already present — skipping"
else
    info "Makefile: injecting ENABLE_NROS_APP CFLAGS + SRCS blocks"

    # Two-step injection:
    #   1. CFLAGS branch BEFORE `LDFLAGS := $(CFLAGS)` so the
    #      preprocessor define propagates to the C compile.
    #   2. LDFLAGS branch AFTER `LDFLAGS := $(CFLAGS)` so the
    #      `-L… -lsentinel_spe_firmware` directive isn't overwritten
    #      by the assignment (`:=` blows away `+=`-additions made
    #      earlier in the file).
    awk '
        /^LDFLAGS := \\/ && !cflags_done {
            print "ifeq ($(ENABLE_NROS_APP), 1)"
            print "\tCFLAGS += -DENABLE_NROS_APP"
            print "endif"
            print ""
            cflags_done = 1
        }
        { print }
        /^[[:space:]]*\$\(CFLAGS\)[[:space:]]*$/ && cflags_done && !ldflags_done {
            print ""
            print "ifeq ($(ENABLE_NROS_APP), 1)"
            print "# LDFLAGS precedes the .o files in cmd_link argv, so a plain"
            print "# -l... is a no-op (single-pass ld pulls only currently-undefined"
            print "# symbols and no .o has been read yet). -Wl,-u,SYM forces an"
            print "# explicit undefined entry so the library scan pulls our entry"
            print "# point + transitive closure."
            print "# `-Wl,-u,vsnprintf` force-pulls our `printf_shim.o`"
            print "# from libsentinel_spe_firmware.a inside the group scan,"
            print "# overriding newlib`s float-aware vsnprintf (which would"
            print "# otherwise drag _dtoa_r + fmaf128 + ~25 KB BTCM)."
            print "LDFLAGS += -L$(SENTINEL_FW_OUT) -Wl,-u,nros_app_rust_entry -Wl,-u,vsnprintf -Wl,-u,printf -Wl,-u,vprintf -Wl,--start-group -lsentinel_spe_firmware -Wl,--end-group"
            print "endif"
            ldflags_done = 1
        }
    ' "$MAKEFILE" > "$MAKEFILE.new"
    mv "$MAKEFILE.new" "$MAKEFILE"

    # SRCS branch — must land BEFORE `objname = ...` / `OBJS := ...`
    # because OBJS captures SRCS by value at that point. The other
    # ENABLE_*_APP SRCS blocks are immediately above this line in the
    # upstream Makefile, so injecting before `objname` puts ours in
    # the same dispatch group.
    awk '
        /^objname[[:space:]]*=/ && !done {
            print "# Phase 11.5.c — autoware-sentinel SPE firmware app."
            print "ifeq ($(ENABLE_NROS_APP), 1)"
            print "SRCS += $(RT_AUX_DIR)/app/nros-app.c"
            print "endif"
            print ""
            done = 1
        }
        { print }
    ' "$MAKEFILE" > "$MAKEFILE.new"
    mv "$MAKEFILE.new" "$MAKEFILE"
fi

# ----------------------------------------------------------------------
# Edit 2: main.c — add nros_app_init() call inside main_task.
# Anchor: the `rtosTaskDelete(NULL);` line that ends main_task.
# ----------------------------------------------------------------------

if grep -q 'ENABLE_NROS_APP' "$MAINC"; then
    info "main.c: ENABLE_NROS_APP already present — skipping"
else
    info "main.c: injecting nros_app_init() forward decl + call"

    # Two-step edit: forward-declare `nros_app_init` at file scope
    # (the BSP builds with `-Wnested-externs -Werror`, so the decl
    # has to live outside main_task), then add the call before
    # `rtosTaskDelete(NULL)` inside main_task.
    awk '
        /^void main_task\(void \*params\)/ && !decl_done {
            print "#if defined(ENABLE_NROS_APP)"
            print "extern void nros_app_init(void);"
            print "#endif"
            print ""
            decl_done = 1
        }
        /^[[:space:]]*rtosTaskDelete\(NULL\);[[:space:]]*$/ && !call_done {
            print ""
            print "#if defined(ENABLE_NROS_APP)"
            print "\tnros_app_init();"
            print "#endif"
            call_done = 1
        }
        { print }
    ' "$MAINC" > "$MAINC.new"
    mv "$MAINC.new" "$MAINC"
fi

# ----------------------------------------------------------------------
# Edit 3: copy the C shim into the BSP's app/ directory.
# ----------------------------------------------------------------------

[ -f "$SHIM_SRC" ] || die "Shim source missing: $SHIM_SRC"
if ! cmp -s "$SHIM_SRC" "$SHIM_DST" 2>/dev/null; then
    info "Staging shim: $SHIM_DST"
    cp "$SHIM_SRC" "$SHIM_DST"
else
    info "Shim already up-to-date: $(basename "$SHIM_DST")"
fi

info "BSP integration edits applied at $BSP"
