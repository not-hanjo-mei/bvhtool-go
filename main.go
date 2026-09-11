// bvhtool — BVH whole-body Y lift + Y yaw transformer.
// Port of bvhtool.py: pivot rotation + continuous Euler unwrapping.
package main

import (
	"bufio"
	"flag"
	"fmt"
	"math"
	"os"
	"strconv"
	"strings"
)

type quat struct{ w, x, y, z float64 }

func qMul(a, b quat) quat {
	return quat{
		w: a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
		x: a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
		y: a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
		z: a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
	}
}

func qAxis(x, y, z, rad float64) quat {
	h := rad * 0.5
	s := math.Sin(h)
	return quat{math.Cos(h), x * s, y * s, z * s}
}

func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

// Extrinsic ZXY (BVH channel order Zrot Xrot Yrot) → quat.
// R = Rz(z)·Rx(x)·Ry(y)
func eulerZXYToQuat(zDeg, xDeg, yDeg float64) quat {
	d2r := math.Pi / 180
	return qMul(qMul(qAxis(0, 0, 1, zDeg*d2r), qAxis(1, 0, 0, xDeg*d2r)), qAxis(0, 1, 0, yDeg*d2r))
}

// quat → extrinsic ZXY degrees [z, x, y].
func quatToEulerZXY(q quat) (zDeg, xDeg, yDeg float64) {
	xRot := math.Asin(clamp(2*(q.y*q.z+q.x*q.w), -1, 1))
	yRot := math.Atan2(2*(q.y*q.w-q.x*q.z), 1-2*(q.x*q.x+q.y*q.y))
	zRot := math.Atan2(2*(q.z*q.w-q.x*q.y), 1-2*(q.x*q.x+q.z*q.z))
	r2d := 180 / math.Pi
	return zRot * r2d, xRot * r2d, yRot * r2d
}

// continuousEuler unwraps principal Euler angles to minimize frame-to-frame jumps.
func continuousEuler(principal [][3]float64, initial [3]float64) [][3]float64 {
	result := make([][3]float64, len(principal))
	prev := initial
	for f, a := range principal {
		b := [3]float64{a[0] + 180, 180 - a[1], a[2] + 180}
		for i := range a {
			a[i] += 360 * math.Round((prev[i]-a[i])/360)
			b[i] += 360 * math.Round((prev[i]-b[i])/360)
		}
		da := (a[0]-prev[0])*(a[0]-prev[0]) + (a[1]-prev[1])*(a[1]-prev[1]) + (a[2]-prev[2])*(a[2]-prev[2])
		db := (b[0]-prev[0])*(b[0]-prev[0]) + (b[1]-prev[1])*(b[1]-prev[1]) + (b[2]-prev[2])*(b[2]-prev[2])
		if db < da {
			result[f] = b
		} else {
			result[f] = a
		}
		prev = result[f]
	}
	for f := range result {
		for i := 0; i < 3; i++ {
			if math.Abs(result[f][i]) < 0.5e-7 {
				result[f][i] = 0
			}
		}
	}
	return result
}

func main() {
	var in, out string
	var liftCm, yawDeg float64

	flag.StringVar(&in, "i", "", "input BVH (required)")
	flag.StringVar(&in, "input", "", "")
	flag.StringVar(&out, "o", "", "output BVH (required)")
	flag.StringVar(&out, "output", "", "")
	flag.Float64Var(&liftCm, "y", 0, "Y lift cm (+=up)")
	flag.Float64Var(&liftCm, "lift-cm", 0, "")
	flag.Float64Var(&yawDeg, "r", 0, "Y yaw degrees (+=left -=right)")
	flag.Float64Var(&yawDeg, "yaw", 0, "")
	flag.Usage = func() {
		fmt.Fprint(os.Stderr, `bvhtool — BVH whole-body transformer

Usage:
  bvhtool -i <input.bvh> -o <output.bvh> [-y 0] [-r 0]

Options:
  -i, --input <file>    input BVH (required)
  -o, --output <file>   output BVH (required)
  -y, --lift-cm <cm>    Y lift, +=up -=down (default: 0)
  -r, --yaw <deg>       Y yaw, +=left -=right (default: 0)

Example:
  bvhtool -i sample.bvh -o out.bvh -y 10 -r 90
`)
	}
	flag.Parse()

	if in == "" || out == "" {
		flag.Usage()
		os.Exit(1)
	}
	if liftCm == 0 && yawDeg == 0 {
		fmt.Fprintln(os.Stderr, "no offset specified (-y and/or -r required)")
		os.Exit(1)
	}

	raw, err := os.ReadFile(in)
	if err != nil {
		fmt.Fprintf(os.Stderr, "read %s: %v\n", in, err)
		os.Exit(1)
	}
	text := strings.TrimPrefix(string(raw), "\ufeff")

	mi := strings.Index(text, "MOTION")
	if mi < 0 {
		fmt.Fprintln(os.Stderr, "no MOTION section")
		os.Exit(1)
	}

	hdr := text[:mi]
	mot := strings.TrimSpace(text[mi:])

	sc := bufio.NewScanner(strings.NewReader(mot))
	sc.Buffer(make([]byte, 1<<20), 1<<20)
	var lines []string
	for sc.Scan() {
		lines = append(lines, sc.Text())
	}

	var frameCount int
	var frameTime float64
	var dataStart int
	for idx, line := range lines {
		t := strings.TrimSpace(line)
		if strings.HasPrefix(t, "Frames:") {
			frameCount, _ = strconv.Atoi(strings.TrimSpace(t[7:]))
		} else if strings.HasPrefix(t, "Frame Time:") {
			frameTime, _ = strconv.ParseFloat(strings.TrimSpace(t[11:]), 64)
			dataStart = idx + 1
		}
	}
	if frameCount == 0 || frameTime == 0 {
		fmt.Fprintln(os.Stderr, "missing Frames/Frame Time")
		os.Exit(1)
	}

	frames := make([][]string, frameCount)
	for i := 0; i < frameCount && dataStart+i < len(lines); i++ {
		frames[i] = strings.Fields(lines[dataStart+i])
		if len(frames[i]) < 6 {
			fmt.Fprintf(os.Stderr, "frame %d: too few channels\n", i)
			os.Exit(1)
		}
	}

	// Root channels: [0]=Xpos [1]=Ypos [2]=Zpos [3]=Zrot [4]=Xrot [5]=Yrot
	start := 0
	if frameCount > 1 {
		z0 := parseFloat(frames[0][3])
		x0 := parseFloat(frames[0][4])
		y0 := parseFloat(frames[0][5])
		if math.Abs(z0) < 1e-7 && math.Abs(x0) < 1e-7 && math.Abs(y0) < 1e-7 {
			start = 1
		}
	}

	pivotX := parseFloat(frames[start][0])
	pivotZ := parseFloat(frames[start][2])

	yawRad := yawDeg * math.Pi / 180
	cosY := math.Cos(yawRad)
	sinY := math.Sin(yawRad)
	qYaw := qAxis(0, 1, 0, yawRad)

	var eulers [][3]float64
	if yawDeg != 0 {
		eulerInit := [3]float64{
			parseFloat(frames[start][3]),
			parseFloat(frames[start][4]),
			parseFloat(frames[start][5]),
		}
		principal := make([][3]float64, frameCount-start)
		for f := start; f < frameCount; f++ {
			qCur := eulerZXYToQuat(parseFloat(frames[f][3]), parseFloat(frames[f][4]), parseFloat(frames[f][5]))
			qNew := qMul(qYaw, qCur)
			z, x, y := quatToEulerZXY(qNew)
			principal[f-start] = [3]float64{z, x, y}
		}
		eulers = continuousEuler(principal, eulerInit)
	}

	for f := start; f < frameCount; f++ {
		fields := frames[f]
		idx := f - start

		if liftCm != 0 {
			fields[1] = strconv.FormatFloat(parseFloat(fields[1])+liftCm, 'g', 15, 64)
		}

		if yawDeg != 0 {
			xPos := parseFloat(fields[0])
			zPos := parseFloat(fields[2])
			dx := xPos - pivotX
			dz := zPos - pivotZ
			fields[0] = strconv.FormatFloat(dx*cosY+dz*sinY+pivotX, 'g', 15, 64)
			fields[2] = strconv.FormatFloat(-dx*sinY+dz*cosY+pivotZ, 'g', 15, 64)
			fields[3] = strconv.FormatFloat(eulers[idx][0], 'f', 7, 64)
			fields[4] = strconv.FormatFloat(eulers[idx][1], 'f', 7, 64)
			fields[5] = strconv.FormatFloat(eulers[idx][2], 'f', 7, 64)
		}

		lines[dataStart+f] = strings.Join(fields, " ")
	}

	f, err := os.Create(out)
	if err != nil {
		fmt.Fprintf(os.Stderr, "create %s: %v\n", out, err)
		os.Exit(1)
	}
	defer f.Close()

	w := bufio.NewWriter(f)
	w.WriteString(hdr)
	for _, line := range lines {
		w.WriteString(line)
		w.WriteByte('\n')
	}
	w.Flush()

	fmt.Fprintf(os.Stderr, "%s -> %s  (y=%g cm, r=%g deg, %d frames, preserved %d)\n",
		in, out, liftCm, yawDeg, frameCount, start)
}

func parseFloat(s string) float64 {
	v, _ := strconv.ParseFloat(s, 64)
	return v
}
