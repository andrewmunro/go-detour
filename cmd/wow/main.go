package main

import (
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net/http"
	"os"

	"github.com/arl/go-detour/detour"
	"github.com/arl/gogeo/f32/d3"
	"github.com/gorilla/mux"
)

func check(err error) {
	if err != nil {
		fmt.Printf("error, %v\n", err)
		os.Exit(-1)
	}
}

func checkStatus(status detour.Status) {
	if detour.StatusFailed(status) {
		check(fmt.Errorf("status failed %s", status.Error()))
	}
}

// Converts from wow coords to detour coords
func FromWowCoords(in d3.Vec3) d3.Vec3 {
	ox := in[0]
	in[0] = in[1]
	in[1] = in[2]
	in[2] = ox
	return in
}

// Coverts from detour coords to wow coords
func ToWowCoords(in d3.Vec3) d3.Vec3 {
	oz := in[2]
	in[2] = in[1]
	in[1] = in[0]
	in[0] = oz
	return in
}

func Vector3ToVec3(in Vector3) d3.Vec3 {
	return d3.NewVec3XYZ(in.X, in.Y, in.Z)
}

func Vec3ToVector3(in d3.Vec3) Vector3 {
	return Vector3{X: in[0], Y: in[1], Z: in[2]}
}

type MMTileHeader struct {
	MMapMagic   uint32
	DTVersion   uint32
	MMapVersion uint32
	Size        uint32
	UsesLiquids byte
	Padding     [3]byte
}

var (
	start    = FromWowCoords(d3.Vec3{-8921.09, -119.135, 82.195})
	end      = FromWowCoords(d3.Vec3{-9448.55, 68.236, 56.3225})
	maxPolys = 256
)

type Nav struct {
	mesh    *detour.NavMesh
	query   *detour.NavMeshQuery
	filter  *detour.StandardQueryFilter
	extents d3.Vec3
}

type Vector3 struct {
	X float32 `json:"x"`
	Y float32 `json:"y"`
	Z float32 `json:"z"`
}

type PathRequest struct {
	Start Vector3 `json:"start"`
	End   Vector3 `json:"end"`
}

func main() {
	nav := NewNav("mmaps/", "000")

	fmt.Println(nav.GetStraightPath(start, end))

	r := mux.NewRouter()
	r.HandleFunc("/path", nav.HandleGetPath).Methods("POST")
	r.HandleFunc("/closest", nav.HandleGetClosestPoints).Methods("POST")

	http.Handle("/", r)

	srv := &http.Server{
		Addr:    ":8080",
		Handler: r,
	}
	srv.ListenAndServe()
}

func (n *Nav) HandleGetPath(w http.ResponseWriter, r *http.Request) {
	var req PathRequest
	err := json.NewDecoder(r.Body).Decode(&req)
	if err != nil {
		w.WriteHeader(400)
		w.Write([]byte(fmt.Sprintf(`{"error": "%s"}`, err)))
		return
	}

	start := FromWowCoords(Vector3ToVec3(req.Start))
	end := FromWowCoords(Vector3ToVec3(req.End))

	path := n.GetStraightPath(start, end)

	vecs := make([]Vector3, len(path))
	for i, vec := range path {
		vecs[i] = Vec3ToVector3(ToWowCoords(vec))
	}

	json.NewEncoder(w).Encode(vecs)
}

func (n *Nav) HandleGetClosestPoints(w http.ResponseWriter, r *http.Request) {
	var req []Vector3
	err := json.NewDecoder(r.Body).Decode(&req)
	if err != nil {
		w.WriteHeader(400)
		w.Write([]byte(fmt.Sprintf(`{"error": "%s"}`, err)))
		return
	}

	res := make([]Vector3, len(req))

	for i, point := range req {
		in := FromWowCoords(Vector3ToVec3(point))
		closest, _ := n.GetClosestPoint(in)
		res[i] = Vec3ToVector3(ToWowCoords(closest))
	}

	json.NewEncoder(w).Encode(res)
}

func NewNav(path, mapId string) *Nav {
	mesh := loadMap(path, mapId)

	status, query := detour.NewNavMeshQuery(mesh, 65535)
	checkStatus(status)

	filter := detour.NewStandardQueryFilter()
	filter.SetIncludeFlags(5)
	filter.SetExcludeFlags(10)

	return &Nav{
		mesh:    mesh,
		query:   query,
		filter:  filter,
		extents: d3.Vec3{6, 6, 6},
	}
}

func (n *Nav) GetClosestPoint(in d3.Vec3) (d3.Vec3, detour.PolyRef) {
	status, poly, point := n.query.FindNearestPoly(in, n.extents, n.filter)
	checkStatus(status)
	if !n.query.AttachedNavMesh().IsValidPolyRef(poly) {
		check(fmt.Errorf("not a valid poly ref"))
	}

	return point, poly
}

// TODO
func (n *Nav) GetSmoothPath(start, end d3.Vec3) []d3.Vec3 {
	path := n.GetStraightPath(start, end)
	if len(path) == 0 {
		return []d3.Vec3{}
	}

	smoothPath := make([]d3.Vec3, 0)

	for i := 0; i < len(path)-1; i++ {
		current := path[i]
		next := path[i+1]

		smoothPath = append(smoothPath, current)

		for {
			iter := current.Lerp(next, 1)
			smoothPath = append(smoothPath, iter)

			if iter.Dist(end) < 0.01 {
				break
			}
		}

	}

	return smoothPath
}

func (n *Nav) GetStraightPath(start, end d3.Vec3) []d3.Vec3 {
	path := n.GetPath(start, end)
	if len(path) == 0 {
		return []d3.Vec3{}
	}

	spath := make([]d3.Vec3, maxPolys)
	for i := range spath {
		spath[i] = d3.NewVec3()
	}

	count, status := n.query.FindStraightPath(start, end, path, spath, nil, nil, int32(detour.StraightPathAreaCrossings&detour.StraightPathAllCrossings))
	checkStatus(status)

	return spath[:count]
}

func (n *Nav) GetPath(start, end d3.Vec3) []detour.PolyRef {
	// Get Start Poly
	status, startRef, _ := n.query.FindNearestPoly(start, n.extents, n.filter)
	checkStatus(status)
	n.query.AttachedNavMesh()
	if !n.query.AttachedNavMesh().IsValidPolyRef(startRef) {
		check(fmt.Errorf("not a valid poly ref"))
	}

	// Get End Poly
	status, endRef, _ := n.query.FindNearestPoly(end, n.extents, n.filter)
	checkStatus(status)
	if !n.query.AttachedNavMesh().IsValidPolyRef(startRef) {
		check(fmt.Errorf("not a valid poly ref"))
	}

	path := make([]detour.PolyRef, maxPolys)

	// Get Path
	count, status := n.query.FindPath(startRef, endRef, start, end, n.filter, path[:])
	checkStatus(status)
	if count == 0 {
		return []detour.PolyRef{}
	}

	return path[:count]
}

func loadMap(path, mapId string) *detour.NavMesh {
	fmt.Println("Loading: " + path + mapId + ".mmap")

	f, err := os.Open(path + mapId + ".mmap")
	check(err)
	defer f.Close()

	params := detour.NavMeshParams{}
	err = binary.Read(f, binary.LittleEndian, &params)
	check(err)

	mesh := detour.NavMesh{}
	status := mesh.Init(&params)

	if detour.StatusFailed(status) {
		check(fmt.Errorf("status failed 0x%x", status))
	}

	ref := detour.TileRef(0)

	for x := 1; x < 64; x++ {
		for y := 1; y < 64; y++ {
			tileMapFileName := fmt.Sprintf("%s%s%d%d.mmtile", path, mapId, x, y)

			if _, err := os.Stat(tileMapFileName); errors.Is(err, os.ErrNotExist) {
				continue
			}

			f, err := os.Open(tileMapFileName)
			check(err)
			defer f.Close()

			header := MMTileHeader{}
			err = binary.Read(f, binary.LittleEndian, &header)
			check(err)

			data := make([]byte, header.Size)
			_, err = io.ReadFull(f, data)
			check(err)

			status, _ = mesh.AddTile(data, ref)
			// fmt.Println("loaded tile ", tileMapFileName, status.Error())
			checkStatus(status)
		}
	}

	return &mesh
}
