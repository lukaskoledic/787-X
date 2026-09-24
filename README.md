# 787 Display Plug-in

This C++ X-Plane plug-in opens three independently controlled windows. All windows use a 4:3 layout:

- **PFD** — left flight/time information column, attitude and tapes above a compass display.
- **ND / EICAS split** — forward arc navigation display beside paired engine indications and a flight-control status row. The lower vertical profile is omitted.
- **Lower display** — switch between **STATUS** and **EFB** from the tabs at the lower right.

Open **Plugins → 787 Displays** and choose a `Show / hide ...` item for each window. Each window uses a plug-in-drawn black surface and draggable title strip; click the `X` to close it. The ND range changes with the mouse wheel. Simulator values are sampled after the flight model and refreshed at 20 Hz. Flight, transponder, SELCAL, tail, and date fields remain `---` because this plug-in does not yet read aircraft identity or date sources.

The windows use `xplm_WindowDecorationSelfDecorated`, so X-Plane supplies no floating-window frame or backing; the plug-in paints an opaque black surface first and draws its own header. NanoVG's pinned OpenGL 2 renderer provides anti-aliased fills, paths, clipping, strokes, and text. Labels use the system Arial TrueType font when available (on macOS, `/System/Library/Fonts/Supplemental/Arial.ttf`); the font is loaded at runtime rather than redistributed with the plug-in. NanoVG's upstream license and source revision are retained under `third_party/nanovg/`. X-Plane's plug-in window API supports OpenGL drawing, including when the simulator is running on its Vulkan or Metal renderer; it does not expose a native Vulkan drawing API for plug-in windows.

On the PFD, the moving blue-sky and brown ground fills span the full attitude region from the speed tape to the vertical-speed side. The horizon bisects the attitude field at zero pitch and bank and follows live pitch and roll. The earth fill uses `#6E3C07`, the dominant RGB value sampled from the supplied reference JPEG's flat ground area; JPEG compression means this is a reproducible image sample, not a verified Boeing color specification. Rounded, translucent gray tapes are drawn above the horizon, and the airspeed scale never labels negative speeds. Flight-director command bars appear when X-Plane reports the director and command bars active.

The PFD barometric indication is always rendered in hPa per the project requirement. It converts the live pilot setting from inHg to hPa, displays `STD 1013 HPA` when standard pressure is selected, and highlights X-Plane's transition-setting warning in amber. Radio altitude and selected radio minimums use live simulator values; the RA/DH indications turn amber when X-Plane asserts the decision-height light. `PULL UP` is red only while X-Plane's GPWS-active annunciator is true. Windshear advisory/caution uses amber and predictive/reactive warning uses red, following X-Plane's documented warning enum. TCAS proximate, traffic-advisory, and resolution-advisory targets are drawn from X-Plane's relative bearing/distance/altitude and threat arrays; a TCAS resolution advisory adds the simulator-provided red/green vertical-speed bands to the VSI. The lower arc can show an 80 NM TERR layer when TERR is enabled in X-Plane; scenery probes update 153 terrain samples once per second and shade them by relative elevation. The TERR state is labeled even when disabled or terrain samples are unavailable.

The terrain layer depicts loaded X-Plane scenery, not a Boeing worldwide obstacle database. It is a terrain-awareness visualization, not a TAWS predictor: the generic X-Plane GPWS flag does not identify whether a warning is terrain, sink rate, or another mode, so the plug-in only shows the `PULL UP` annunciation when that flag is active. The X-Plane warning and TCAS inputs follow X-Plane's public datarefs, not a Boeing AFDS/TAWS integration; absent datarefs remain inactive/placeholders. 787 references describe the terrain overlay as relative-altitude color contours/dot patterns and show TERR as selectable independently from weather. Exact OEM TAWS thresholds, alert logic, and terrain database content still require aircraft/add-on sources.

## PFD flight-mode annunciations

The PFD FMA is organized as **A/T**, **ROLL**, and **PITCH**. Active modes are green, armed modes are white, and a newly active mode receives a green outline for ten seconds. The plugin reads X-Plane's autopilot state; it never engages or changes a mode itself. For example, LNAV appears from the GPSS/FMS navigation state, while VNAV SPD, VNAV PTH, and VNAV ALT are tied to X-Plane's corresponding VNAV and vertical-mode state.

This is a simulator-backed approximation of the 787 presentation. X-Plane's generic autopilot state does not expose every Boeing AFDS mode, fault, or aircraft-specific transition. An aircraft add-on with its own AFDS logic may require its own documented datarefs before every annunciation can follow its state exactly.

## Build on macOS

With the extracted X-Plane Plugin SDK at `/Users/lukas/Desktop/SDK`:

```sh
cmake -S plugins/787PFD -B /tmp/787-displays-build -DXPLANE_SDK="/Users/lukas/Desktop/SDK"
cmake --build /tmp/787-displays-build
```

The build writes `mac.xpl` to `plugins/787PFD/64/`. Install the plug-in folder in X-Plane 12's `Resources/plugins` directory, then enable it in X-Plane if needed.

## Fidelity boundary

The supplied images are the visual reference for this prototype. It uses X-Plane flight and engine datarefs, with placeholder values when a source is unavailable. Arial improves legibility but is a presentation choice rather than a verified copy of Boeing's complete display typography. The drawings are not a verified reproduction of Boeing's certified display software, alerting rules, flight-management data, or aircraft-specific modes. The installed aircraft and its datarefs determine which indications can be populated.

Research references: [Boeing's licensed flight-training manuals page](https://services.boeing.com/training-solutions/flight-training/licensed-manuals) describes access to current Boeing FCOM/QRH/FCTM material; the public [flight-deck information management study](https://rosap.ntl.bts.gov/view/dot/77507/dot_77507_DS1.pdf) discusses 787 PFD/mini-map indications; [QualityWings Ultimate 787 Collection User's Manual](https://www.qualitywingsim.com/files/ultimate_787_collection/docs/QualityWings%20-%20Ultimate%20787%20Collection%20Users%20Manual.pdf) provides public 787 simulation references. For simulator behavior, see Laminar Research's [autopilot dataref guide](https://developer.x-plane.com/article/accessing-the-x-plane-autopilot-from-datarefs/), [VNAV behavior guide](https://developer.x-plane.com/article/autopilot-vnav-modes-and-planemaker-settings/), [GPSS/LNAV guide](https://developer.x-plane.com/article/track-to-intercept-making-sense-of-lnav-and-locapp-modes-in-11-10/), [flight director/autothrottle datarefs](https://developer.x-plane.com/article/flight-director-and-autothrottle-datarefs/), [terrain probing API](https://developer.x-plane.com/sdk/XPLMScenery/), and [TCAS resolution-advisory interface](https://developer.x-plane.com/article/tcas-resolution-advisories-and-plugin-traffic/).
