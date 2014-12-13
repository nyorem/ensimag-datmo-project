# Read files
frames = scan("frames.txt")
speeds = scan("speeds.txt")

# Output to png
png("speed_estimation.png");
plot(frames, speeds)
lines(frames, speeds)
dev.off();
