# FusionCore: 100 Deliverables to ROSCon (June 11 – October 5, 2026)

Hardware build excluded (separate playbook). Everything else that moves you from $0 to first commercial conversations, phased against your actual calendar.

**The four things everything below serves:** (1) UniNa case study, (2) the hardware video shipped by ~Aug 25, (3) ROSCon converted into priced proposals, (4) a store that's actually open when traffic arrives.

> **Status key:** [x] done | [ ] not started | [~] partial

---

## PHASE 1 — June 11 → July 7

### Workstream A: The UniNa deal (highest priority, this week)

- [~] 1. Reply to UniNa's open GitHub issue today with the fix/guidance, and in the same comment offer a 45-min video call — move support off async. *(reply sent, video call not offered yet)*
- [ ] 2. Hold the call with Pasquale: get their full pipeline working end-to-end (Xsens MTi-680G + RK4 bicycle model + kiss-icp plan), white-glove level.
- [x] 3. Make the sponsorship ask explicitly: dedicated technical support through competition season in exchange for (a) joint case study, (b) technical-sponsor listing (logo on car/design docs/socials), (c) rosbags from real runs. *(sent via GitHub)*
- [ ] 4. Follow up the call with a one-page written sponsorship summary email so the agreement is concrete and forwardable to their team lead.
- [ ] 5. Request 2–3 rosbags from real car runs (RTK GNSS + IMU + encoder topics) — your free ground-truth data leg.
- [ ] 6. Write the certified config: Xsens MTi-680G + bicycle-model odometry FusionCore parameter file, tested against their bags, committed to the repo under `configs/certified/`.
- [ ] 7. Open a tracking doc for the case study: their FSAE Italy 2024 GPS-corruption failure story, the integration timeline, quotes as they come.

### Workstream B: Repair & credibility (the Tom Moore problem)

- [x] 8. Pull every public comparative claim about robot_localization (README, docs, arXiv abstract page, old posts) into one audit list. *(only vs-robot-localization.md audited so far)*
- [x] 9. Cross-check each claim against Tom Moore's posted corrections; fix or soften anything inaccurate or unfair. Commit the changes. *(noise covariance row fixed and pushed)*
- [x] 10. Rewrite the README positioning section to lead with what FusionCore *does* (the ROSCon reviewer feedback) rather than what robot_localization doesn't.
- [x] 11. Draft the robot_localization fairness config for the future video comparison: a good-faith, documented r_l setup, committed to the repo with an open invitation for PRs to improve it.
- [x] 12. Write the fairness disclaimer text that will accompany every published comparison (video description, docs page, Discourse post).

### Workstream C: Open the store (commercial infrastructure)

- [x] 13. Write the support/pricing page: Integration ($750 fixed) + Production Support ($500/mo). Published on the docs site. *(docs/support.md)*
- [x] 14. Add a "Commercial Support" link to the README and docs nav.
- [x] 15. Create a proposal template (2 pages: their problem, scope, SLA terms, price) so a ROSCon conversation can become a sent proposal in under an hour.
- [ ] 16. Set up a simple intake: Calendly or email alias (support@ / hello@) on the pricing page.
- [ ] 17. Write the Q1 2027 decision criteria in a private note: "2 orgs paying anything recurring by March 31, 2027, or FusionCore re-scopes to maintenance-mode prestige OSS." Date it. This is your anti-burnout forcing function.

### Workstream D: Convert existing goodwill (the gratitude loop)

- [x] 18. Create `ADOPTERS.md` in the repo.
- [x] 19. Email Michał Bednarek: permission to list the factory robot deployment in ADOPTERS.md + use his testimonial on the site; ask if any photo/clip of the robot is shareable.
- [x] 20. Email Sam (Agroecology Lab): same ask, plus offer help finishing the integration — an in-progress ag deployment is your ICP in miniature.
- [x] 21. Email Russ Hall: thank-you, ADOPTERS.md ask, and ask his Andino deployment timeline — his $150 deserves a check-in, and his rig mirrors your demo build.
- [x] 22. Add a "Using FusionCore in production? Get listed" line to the README pointing at ADOPTERS.md.
- [x] 23. Backfill Michał's testimonial onto the docs landing page (index.md).

### Workstream E: SEO & LLM corpus (start the compounding asset)

- [x] 24. Mine robot_localization's closed issues: list the 15 most-recurring failure patterns (datum confusion, covariance explosion, GPS spikes near structures, delayed-measurement breakage, UTM zone boundaries, etc.).
- [x] 25. Write troubleshooting page #1: "ROS 2 GPS position jumping near buildings — diagnosis and fixes" (covers r_l *and* FusionCore honestly).
- [x] 26. Write troubleshooting page #2: "Handling delayed GPS measurements in ROS 2 sensor fusion."
- [x] 27. Add `llms.txt` to the docs site; verify docs render cleanly as markdown for crawlers.
- [x] 28. Run the LLM benchmark: ask Claude, GPT, and Gemini "best ROS 2 localization package with GPS outlier rejection" — screenshot results, save as baseline to re-test in October.
- [ ] 29. Set up Google Search Console on the docs site; confirm indexing of the new pages.
- [x] 30. Create the KPI sheet (one spreadsheet, weekly row): stars, clones, docs impressions/clicks, qualified conversations, proposals sent, MRR. Fill week 1.

### Workstream F: Community presence (unburned channels)

- [ ] 31. Create your Robotics Stack Exchange account (or dust it off); answer 2 localization/EKF/GPS questions this phase — genuinely helpful, link only when relevant.
- [ ] 32. Join ROS Discord + any ag-robotics/OpenMower Discords; introduce yourself in a help capacity, answer 2 questions.
- [ ] 33. Reserve the Twitter/X handle (@fusioncore_ros or similar), set up the profile, follow the robotics scene — post nothing yet; the video is the debut.
- [x] 34. Email Mathieu Labbé: thank him for the rtabmap demo merge, give a heads-up that a hardware demo video is coming in August, ask if he'd be open to sharing it if he finds it useful.
- [x] 35. Email Stereolabs evaluation contact: offer direct support for their comparison (Wolf / TIER IV EagleEye / r_l), ask what data or configs would help, mention the August dataset release.

---

## PHASE 2 — July 8 → 24

> **TRIP CANCELLED — items 36–38 not applicable. Full sprint capacity available through July.**

- [x] 36. ~~Pin a repo notice + GitHub status: "Maintainer traveling July 8–24."~~ N/A
- [x] 37. ~~Set the support email autoresponder.~~ N/A
- [x] 38. ~~Issues triage only if connectivity allows.~~ N/A

---

## PHASE 3 — July 25 → Aug 24 (the production sprint)

### Workstream G: Field campaign

- [ ] 39. Scout the torture course on foot with a phone GNSS status app: open-sky park segment, building-wall multipath pass, underpass dropout, loop-closure start/end mark. Confirm 7–8am weekend viability.
- [ ] 40. Write the field-day checklist: topics to record, `ros2 bag info` verification step, battery count, tape measure, phone tripod, chalk.
- [ ] 41. Write the bag-recording launch file: IMU raw, encoder odom, NavSatFix, GPS status/diagnostics, tf_static — test it end-to-end in the driveway.
- [ ] 42. Execute field day: 3 full recorded runs minimum, phone footage of each, loop-closure tape measurement on camera.
- [ ] 43. Hold a backup field-day slot the following weekend (weather/hardware insurance).
- [ ] 44. Run offline replays: same bags through FusionCore (shipped defaults) and robot_localization (the published fairness config from #11).
- [ ] 45. Compute metrics with `evo`: loop-closure error both filters, max excursion during multipath pass, dropout drift, Pi 4 per-cycle timing histogram.
- [ ] 46. Cross-check results against your NCLT numbers — one consistent quantitative story across both datasets.

### Workstream H: The video (two cuts)

- [ ] 47. Write the main-cut script (5–8 min): cold open on the multipath split-screen, no intro fluff, apt command on screen by 1:30, fairness disclaimer spoken aloud, "replay it yourself" close.
- [ ] 48. Record RViz/screen captures (OBS) of the replays with GPS fix-status overlay.
- [ ] 49. Edit the main cut in DaVinci Resolve.
- [ ] 50. Edit the 60-second vertical short cut (multipath split-screen + apt command) — this is the Twitter debut and the lightning-talk core.
- [ ] 51. Create the YouTube channel (FusionCore branding), thumbnail, title optimized for search ("ROS 2 GPS multipath sensor fusion — real hardware torture test").
- [ ] 52. Get one trusted reviewer (Michał?) to watch the main cut before publishing — fairness and clarity check.

### Workstream I: The dataset & writeup

- [ ] 53. Package the field bags + both configs + replay script + README into a Zenodo deposit; mint the DOI.
- [ ] 54. Write the docs-site companion post: full writeup with metrics tables, methodology, fairness statement, links to bags — this is also SEO page #3.
- [ ] 55. Write troubleshooting pages #4 and #5 from the failure-pattern list (item 24).
- [ ] 56. Convert every issue you solved in June–August into short docs entries (the issue-to-content loop) — minimum 3.

### Workstream J: Benchmark harness

- [ ] 57. Spin the NCLT benchmark scripts into a public repo: `fusioncore-benchmarks` — one command runs any filter against the 12 sequences.
- [ ] 58. Include robot_localization and (if feasible) fuse as pre-wired baselines; document how to add your own filter.
- [ ] 59. README the results table with reproduction instructions; link from main repo.

### Workstream K: Nav2 & ecosystem

- [ ] 60. Write the Nav2-integration tutorial and submit it as a docs/community-tutorial PR to the Nav2 docs repo.
- [ ] 61. Finish/ship whatever is blocking OpenMowerNext PR #45.
- [~] 62. Confirm the Jazzy apt sync went live; verify `sudo apt install ros-jazzy-fusioncore-ros` on a clean 24.04 container. *(rosdistro PR #51837 approved, held for sync — not live yet)*
- [ ] 63. Tag v0.3.1 with a clean CHANGELOG if any fixes accumulated; no new features.
- [ ] 64. Add `CITATION.cff` to the repo (arXiv + Zenodo DOIs) so academic users cite you properly.
- [ ] 65. UniNa check-in call #2: progress, blockers, collect quotes and any car photos/footage for the case study.
- [ ] 66. Draft the UniNa case study (publish-ready, pending their sign-off): "How a podium FSAE driverless team replaced their custom UKF."

---

## PHASE 4 — Aug 25 → 31 (launch week — strict sequence)

- [ ] 67. Day 1: publish the YouTube main cut + docs writeup + Zenodo dataset simultaneously.
- [ ] 68. Day 1: first tweet — the 60-second cut, apt command, link to full video. Pin it.
- [ ] 69. Day 1: LinkedIn post (the journey + the result).
- [ ] 70. Day 2: the coordinated re-entry post — ROS Discourse and r/ROS, same day, "gift not ask" framing: dataset + reproducible benchmark + video + v0.3 + apt news + UniNa line, all in ONE post.
- [ ] 71. Day 2–3: megaphone emails with the link — Mathieu Labbé, Michał, Stereolabs, u-blox developer relations, SparkFun/Adafruit content teams.
- [ ] 72. Day 2–3: send to UniNa with ready-to-post social copy so their team can share to their LinkedIn following.
- [ ] 73. Day 4: answer every comment/reply across all channels within 24h — launch week responsiveness IS the marketing.
- [ ] 74. Day 5: re-run the LLM benchmark (item 28) — log any movement.
- [ ] 75. Email Jens Ogorek (Feldfreund) ONE final time — new artifact, new email: just the video link + "thought of your Septentrio setup during the multipath segment."
- [ ] 76. Publish the UniNa case study if signed off (else hold for their competition timing).
- [ ] 77. Update KPI sheet; screenshot GitHub traffic for the week (your before/after).
- [ ] 78. Submit the lightning-talk lottery entry / confirm the at-conference lottery process.

---

## PHASE 5 — Sept 1 → ROSCon eve (conversion prep)

### Workstream L: Talk & materials

- [ ] 79. Build the lightning talk: 2–3 min, structure = 30s torture-test footage → one metrics slide (NCLT 10/12, loop-closure numbers, <1ms Pi histogram) → full-screen `sudo apt install ros-jazzy-fusioncore-ros`.
- [ ] 80. Rehearse it to a hard 2:30, ten times, once in front of a human (or recorded and reviewed).
- [ ] 81. Order stickers with the apt command + logo (~$80, StickerMule or local Hamilton print shop) — 200 count.
- [ ] 82. Simple business cards: name, "FusionCore maintainer," QR to the video, the apt command. 100 count.
- [ ] 83. Load the phone: 60-second cut downloaded offline, plays in 2 taps. This is your hallway demo.
- [ ] 84. Prep the "bring me your bag file" kit: laptop with FusionCore + a quick bag-inspection script ready to run a 10-minute localization diagnosis from anyone's rosbag.

### Workstream M: Targeting

- [ ] 85. Scrape the published ROSCon speaker/sponsor list; build the 15-name target sheet: autonomy/localization leads at ag/inspection/delivery/mining companies, Nav2 maintainers (Steve Macenski #1), Stereolabs attendees, sensor-vendor application engineers (Xsens, u-blox, Septentrio, VectorNav booths).
- [ ] 86. For each of the 15: one line on why FusionCore matters to *them specifically* — your opener.
- [ ] 87. Write the Tom Moore approach plan: respectful intro, credit r_l's influence, zero comparison talk unless he raises it. Goal: adversary → neutral.
- [ ] 88. Write the 3 fleet-operator discovery questions (Path 2 validation): "How do you monitor localization health across robots today? What happens when one drifts? Who gets paged?" — ask every fleet person you meet, log answers.
- [ ] 89. Post a casual meetup to the ROSCon Slack/Discord: "Outdoor localization beers, [bar near venue], Tuesday 7pm — bring your GPS horror stories." You're local; you're the host.
- [ ] 90. Block the full conference + one buffer day after in your calendar; tell work in advance.
- [ ] 91. Set up a contact-capture system: notes app template (name, company, pain, follow-up action) — fill it within 5 minutes of every conversation, every time.
- [ ] 92. Pre-draft the follow-up email skeleton so post-conference sends take 5 minutes each.
- [ ] 93. Pre-draft the proposal-attached follow-up variant for anyone who shows budget-holder pain.

---

## PHASE 6 — ROSCon days + two weeks after (the actual conversion)

- [ ] 94. Conference daily routine: morning = 3 target-list approaches before talks start; hallway track > sessions; demo the clip whenever localization comes up; sticker every laptop that says yes; log every contact same-day.
- [ ] 95. Deliver the lightning talk if the lottery hits; if it doesn't, do the 2:30 pitch one-on-one at least 15 times across the conference.
- [ ] 96. Run the "bag file diagnosis" offer publicly — Slack channel + in person.
- [ ] 97. Host the meetup (#89); your only job there is asking questions and logging pain points.
- [ ] 98. Within 5 days post-conference: every follow-up email sent, personalized to the logged conversation; proposals attached for the 3+ hottest.
- [ ] 99. Within 10 days: publish the ROSCon recap post ("what 50 robotics engineers told me about their localization pain").
- [ ] 100. October 1 scorecard: ≥10 qualified ICP conversations logged, ≥3 proposals sent, ≥1 in active negotiation, docs impressions trending up, LLM benchmark improved.

---

## Standing weekly rhythm (runs underneath everything, all phases)

- Every issue answered <24h (this is the product now, not just support)
- 2 Stack Exchange / Discord answers per week
- KPI sheet updated Sundays, 10 minutes
- Zero feature work that isn't blocking UniNa, the video, or a paying prospect

## The dependencies that can slip everything

- **UniNa sponsorship deal (items 1–7)** gates the case study, the rosbags, and half of launch week → this week.
- **Field day (item 42)** gates the video, launch week, and the lightning talk → protect one July weekend + backup.
- **Pricing page (item 13)** gates every conversion downstream. DONE.
