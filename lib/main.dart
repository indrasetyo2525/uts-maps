import 'dart:math';
import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';

void main() {
  runApp(const SmartLogisticsApp());
}

// ==========================================
// 1. MODELS
// ==========================================
class LocationPoint {
  final int index;
  final double latitude;
  final double longitude;
  final String label;

  LocationPoint({
    required this.index,
    required this.latitude,
    required this.longitude,
    required this.label,
  });

  LatLng get latLng => LatLng(latitude, longitude);
  String get shortCode => 'P$index';
}

class RouteResult {
  final String algorithmName;
  final List<LocationPoint> orderedPoints;
  final double totalDistanceKm;
  final Color lineColor;

  RouteResult({
    required this.algorithmName,
    required this.orderedPoints,
    required this.totalDistanceKm,
    required this.lineColor,
  });

  String get sequenceText =>
      orderedPoints.map((p) => '${p.shortCode} (${p.label})').join(' -> ');

  String get compactSequenceText =>
      orderedPoints.map((p) => p.shortCode).join(' -> ');
}

// ==========================================
// 2. SERVICES (Distance & Routing Algorithms)
// ==========================================
class DistanceCalculator {
  static const double earthRadiusKm = 6371.0;

  static double _degreesToRadians(double degrees) {
    return degrees * pi / 180.0;
  }

  static double haversineKm(LocationPoint p1, LocationPoint p2) {
    final double dLat = _degreesToRadians(p2.latitude - p1.latitude);
    final double dLon = _degreesToRadians(p2.longitude - p1.longitude);

    final double lat1Rad = _degreesToRadians(p1.latitude);
    final double lat2Rad = _degreesToRadians(p2.latitude);

    final double a = pow(sin(dLat / 2), 2) +
        cos(lat1Rad) * cos(lat2Rad) * pow(sin(dLon / 2), 2);

    final double c = 2 * asin(sqrt(a));
    return earthRadiusKm * c;
  }

  static double calculateTotalRouteDistance(List<LocationPoint> route) {
    if (route.length < 2) return 0.0;
    double total = 0.0;
    for (int i = 0; i < route.length - 1; i++) {
      total += haversineKm(route[i], route[i + 1]);
    }
    return total;
  }
}

class RoutingService {
  static RouteResult buildSequentialRoute(List<LocationPoint> points) {
    return RouteResult(
      algorithmName: 'Rute Sesuai Urutan Input',
      orderedPoints: List.from(points),
      totalDistanceKm: DistanceCalculator.calculateTotalRouteDistance(points),
      lineColor: Colors.orange.shade700,
    );
  }

  static RouteResult solveDijkstra(List<LocationPoint> points) {
    final int n = points.length;
    if (n < 2) {
      return RouteResult(
        algorithmName: 'Rute Terpendek Dijkstra',
        orderedPoints: List.from(points),
        totalDistanceKm: 0.0,
        lineColor: Colors.blue.shade800,
      );
    }

    List<List<double>> graph = List.generate(
      n,
      (i) => List.generate(
        n,
        (j) => i == j ? 0.0 : DistanceCalculator.haversineKm(points[i], points[j]),
      ),
    );

    List<bool> visitedRoute = List.filled(n, false);
    List<LocationPoint> optimalPath = [];

    int currentNode = 0;
    visitedRoute[currentNode] = true;
    optimalPath.add(points[currentNode]);

    while (optimalPath.length < n) {
      List<double> dist = _runSingleSourceDijkstra(graph, currentNode);
      int nextNode = -1;
      double minDist = double.infinity;

      for (int i = 0; i < n; i++) {
        if (!visitedRoute[i] && dist[i] < minDist) {
          minDist = dist[i];
          nextNode = i;
        }
      }

      if (nextNode != -1) {
        visitedRoute[nextNode] = true;
        optimalPath.add(points[nextNode]);
        currentNode = nextNode;
      }
    }

    double totalKm = DistanceCalculator.calculateTotalRouteDistance(optimalPath);
    return RouteResult(
      algorithmName: 'Rute Terpendek Dijkstra',
      orderedPoints: optimalPath,
      totalDistanceKm: totalKm,
      lineColor: Colors.blue.shade800,
    );
  }

  static List<double> _runSingleSourceDijkstra(
      List<List<double>> graph, int source) {
    int n = graph.length;
    List<double> dist = List.filled(n, double.infinity);
    List<bool> settled = List.filled(n, false);

    dist[source] = 0.0;

    for (int count = 0; count < n - 1; count++) {
      int u = -1;
      double minVal = double.infinity;
      for (int i = 0; i < n; i++) {
        if (!settled[i] && dist[i] <= minVal) {
          minVal = dist[i];
          u = i;
        }
      }

      if (u == -1) break;
      settled[u] = true;

      for (int v = 0; v < n; v++) {
        if (!settled[v] &&
            graph[u][v] > 0 &&
            dist[u] != double.infinity &&
            dist[u] + graph[u][v] < dist[v]) {
          dist[v] = dist[u] + graph[u][v];
        }
      }
    }
    return dist;
  }

  static RouteResult solveAStar(List<LocationPoint> points) {
    final int n = points.length;
    if (n < 2) {
      return RouteResult(
        algorithmName: 'Algoritma A* Search',
        orderedPoints: List.from(points),
        totalDistanceKm: 0.0,
        lineColor: Colors.purple.shade700,
      );
    }

    List<LocationPoint> bestPath = [];
    double minCost = double.infinity;
    List<_AStarState> queue = [];
    int startMask = 1;

    queue.add(_AStarState(
      visitedMask: startMask,
      currentIndex: 0,
      path: [points[0]],
      gCost: 0.0,
      hCost: _calculateHeuristic(0, startMask, points),
    ));

    while (queue.isNotEmpty) {
      queue.sort((a, b) => a.fCost.compareTo(b.fCost));
      _AStarState current = queue.removeAt(0);

      if (current.path.length == n) {
        if (current.gCost < minCost) {
          minCost = current.gCost;
          bestPath = List.from(current.path);
        }
        continue;
      }

      for (int nextIdx = 0; nextIdx < n; nextIdx++) {
        if ((current.visitedMask & (1 << nextIdx)) == 0) {
          int newMask = current.visitedMask | (1 << nextIdx);
          double stepCost = DistanceCalculator.haversineKm(
            points[current.currentIndex],
            points[nextIdx],
          );
          double newGCost = current.gCost + stepCost;
          double newHCost = _calculateHeuristic(nextIdx, newMask, points);

          queue.add(_AStarState(
            visitedMask: newMask,
            currentIndex: nextIdx,
            path: [...current.path, points[nextIdx]],
            gCost: newGCost,
            hCost: newHCost,
          ));
        }
      }

      if (queue.length > 1000) {
        queue.sort((a, b) => a.fCost.compareTo(b.fCost));
        queue = queue.sublist(0, 500);
      }
    }

    if (bestPath.isEmpty) bestPath = List.from(points);
    double totalKm = DistanceCalculator.calculateTotalRouteDistance(bestPath);

    return RouteResult(
      algorithmName: 'Algoritma A* Search',
      orderedPoints: bestPath,
      totalDistanceKm: totalKm,
      lineColor: Colors.purple.shade700,
    );
  }

  static double _calculateHeuristic(
      int currentIndex, int visitedMask, List<LocationPoint> points) {
    double h = 0.0;
    for (int i = 0; i < points.length; i++) {
      if ((visitedMask & (1 << i)) == 0) {
        h += DistanceCalculator.haversineKm(points[currentIndex], points[i]);
        break;
      }
    }
    return h;
  }
}

class _AStarState {
  final int visitedMask;
  final int currentIndex;
  final List<LocationPoint> path;
  final double gCost;
  final double hCost;

  _AStarState({
    required this.visitedMask,
    required this.currentIndex,
    required this.path,
    required this.gCost,
    required this.hCost,
  });

  double get fCost => gCost + hCost;
}

// ==========================================
// 3. MAIN APP & UI WIDGETS
// ==========================================
class SmartLogisticsApp extends StatelessWidget {
  const SmartLogisticsApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Smart Logistics Routing - Indrasetyo',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.indigo),
        useMaterial3: true,
      ),
      home: const RoutingHomePage(),
    );
  }
}

class RoutingHomePage extends StatefulWidget {
  const RoutingHomePage({super.key});

  @override
  State<RoutingHomePage> createState() => _RoutingHomePageState();
}

class _RoutingHomePageState extends State<RoutingHomePage> {
  final MapController _mapController = MapController();
  final LatLng _currentCenter = const LatLng(-6.2088, 106.8456);

  final List<LocationPoint> _points = [];
  RouteResult? _sequentialResult;
  RouteResult? _dijkstraResult;
  RouteResult? _alternativeResult;
  RouteResult? _activeRoute;

  final TextEditingController _latController = TextEditingController();
  final TextEditingController _lngController = TextEditingController();
  final TextEditingController _labelController = TextEditingController();

  void _reindexPoints() {
    for (int i = 0; i < _points.length; i++) {
      _points[i] = LocationPoint(
        index: i + 1,
        latitude: _points[i].latitude,
        longitude: _points[i].longitude,
        label: _points[i].label,
      );
    }
  }

  void _addPoint(double lat, double lng, String label) {
    if (_points.length >= 10) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('Maksimal 10 titik koordinat tujuan!'),
          backgroundColor: Colors.orange,
        ),
      );
      return;
    }

    setState(() {
      _points.add(
        LocationPoint(
          index: _points.length + 1,
          latitude: lat,
          longitude: lng,
          label: label,
        ),
      );
      _clearRouteResults();
    });
    _mapController.move(LatLng(lat, lng), 13.0);
  }

  void _removePoint(int idx) {
    setState(() {
      _points.removeAt(idx);
      _reindexPoints();
      _clearRouteResults();
    });
  }

  void _clearRouteResults() {
    _sequentialResult = null;
    _dijkstraResult = null;
    _alternativeResult = null;
    _activeRoute = null;
  }

  void _loadSamplePreset() {
    setState(() {
      _points.clear();
      _points.addAll([
        LocationPoint(
            index: 1,
            latitude: -6.2088,
            longitude: 106.8456,
            label: 'Gudang Pusat'),
        LocationPoint(
            index: 2,
            latitude: -6.1754,
            longitude: 106.8650,
            label: 'Outlet Cempaka'),
        LocationPoint(
            index: 3,
            latitude: -6.2250,
            longitude: 106.8000,
            label: 'Pelanggan D'),
        LocationPoint(
            index: 4,
            latitude: -6.1805,
            longitude: 106.8284,
            label: 'Hub Barat'),
      ]);
      _clearRouteResults();
    });
    _mapController.move(const LatLng(-6.1970, 106.8350), 12.5);
  }

  void _submitFormInput() {
    final double? lat = double.tryParse(_latController.text.trim());
    final double? lng = double.tryParse(_lngController.text.trim());
    final String label = _labelController.text.trim();

    if (lat == null || lng == null || label.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('Harap isi Latitude, Longitude, dan Keterangan Lokasi!'),
          backgroundColor: Colors.redAccent,
        ),
      );
      return;
    }

    if (lat < -90 || lat > 90 || lng < -180 || lng > 180) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('Rentang koordinat tidak valid!'),
          backgroundColor: Colors.redAccent,
        ),
      );
      return;
    }

    _addPoint(lat, lng, label);
    _latController.clear();
    _lngController.clear();
    _labelController.clear();
    FocusScope.of(context).unfocus();
  }

  void _showAddPointDialog(LatLng tappedLatLng) {
    final TextEditingController dialogLabelCtrl = TextEditingController();
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Tambah Titik dari Peta'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Koordinat:\nLat: ${tappedLatLng.latitude.toStringAsFixed(6)}\nLng: ${tappedLatLng.longitude.toStringAsFixed(6)}',
              style: const TextStyle(fontSize: 12, fontFamily: 'monospace'),
            ),
            const SizedBox(height: 12),
            TextField(
              controller: dialogLabelCtrl,
              autofocus: true,
              decoration: const InputDecoration(
                labelText: 'Keterangan Lokasi',
                hintText: 'Contoh: Toko Sumber Rejeki',
                border: OutlineInputBorder(),
              ),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('Batal'),
          ),
          ElevatedButton(
            onPressed: () {
              final text = dialogLabelCtrl.text.trim();
              if (text.isNotEmpty) {
                _addPoint(tappedLatLng.latitude, tappedLatLng.longitude, text);
                Navigator.pop(ctx);
              }
            },
            child: const Text('Simpan Titik'),
          ),
        ],
      ),
    );
  }

  void _showMarkerInfoWindow(LocationPoint point) {
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        title: Row(
          children: [
            Icon(Icons.location_on,
                color: point.index == 1 ? Colors.green : Colors.red),
            const SizedBox(width: 8),
            Expanded(child: Text('${point.shortCode}: ${point.label}')),
          ],
        ),
        content: Text(
          'Status: ${point.index == 1 ? "Titik Keberangkatan Awal" : "Destinasi Pengiriman"}\n'
          'Latitude: ${point.latitude.toStringAsFixed(6)}\n'
          'Longitude: ${point.longitude.toStringAsFixed(6)}',
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('Tutup'),
          ),
        ],
      ),
    );
  }

  bool _validateMinimumPoints() {
    if (_points.length < 3) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('Masukkan minimal 3 titik koordinat terlebih dahulu!'),
          backgroundColor: Colors.redAccent,
        ),
      );
      return false;
    }
    return true;
  }

  void _runSequentialRoute() {
    if (!_validateMinimumPoints()) return;
    setState(() {
      _sequentialResult = RoutingService.buildSequentialRoute(_points);
      _activeRoute = _sequentialResult;
    });
  }

  void _runDijkstraRoute() {
    if (!_validateMinimumPoints()) return;
    setState(() {
      _dijkstraResult = RoutingService.solveDijkstra(_points);
      _activeRoute = _dijkstraResult;
    });
  }

  void _runAlternativeRoute() {
    if (!_validateMinimumPoints()) return;
    setState(() {
      _alternativeResult = RoutingService.solveAStar(_points);
      _activeRoute = _alternativeResult;
    });
  }

  void _resetAll() {
    setState(() {
      _points.clear();
      _clearRouteResults();
    });
  }

  @override
  Widget build(BuildContext context) {
    final List<LatLng> polylineCoordinates =
        _activeRoute?.orderedPoints.map((p) => p.latLng).toList() ?? [];

    return Scaffold(
      appBar: AppBar(
        title: const Text(
          'Smart Logistics Routing — Indrasetyo',
          style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
        ),
        backgroundColor: Colors.indigo,
        foregroundColor: Colors.white,
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh),
            tooltip: 'Reset Semua',
            onPressed: _resetAll,
          ),
        ],
      ),
      body: SingleChildScrollView(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            SizedBox(
              height: 340,
              child: Stack(
                children: [
                  FlutterMap(
                    mapController: _mapController,
                    options: MapOptions(
                      initialCenter: _currentCenter,
                      initialZoom: 13.0,
                      onPositionChanged: (camera, hasGesture) {},
                      onTap: (tapPosition, latLng) {
                        _showAddPointDialog(latLng);
                      },
                    ),
                    children: [
                      TileLayer(
                        urlTemplate:
                            'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
                        userAgentPackageName:
                            'com.uts.indrasetyo.smartlogistics',
                      ),
                      if (polylineCoordinates.length >= 2)
                        PolylineLayer(
                          polylines: [
                            Polyline(
                              points: polylineCoordinates,
                              strokeWidth: 5.0,
                              color: _activeRoute!.lineColor,
                            ),
                          ],
                        ),
                      MarkerLayer(
                        markers: _points.map((point) {
                          final bool isStart = point.index == 1;
                          return Marker(
                            point: point.latLng,
                            width: 95,
                            height: 75,
                            child: GestureDetector(
                              onTap: () => _showMarkerInfoWindow(point),
                              child: Column(
                                mainAxisSize: MainAxisSize.min,
                                children: [
                                  Container(
                                    padding: const EdgeInsets.symmetric(
                                        horizontal: 6, vertical: 2),
                                    decoration: BoxDecoration(
                                      color: isStart
                                          ? Colors.green.shade800
                                          : Colors.red.shade700,
                                      borderRadius: BorderRadius.circular(6),
                                      boxShadow: const [
                                        BoxShadow(
                                            color: Colors.black26,
                                            blurRadius: 3)
                                      ],
                                    ),
                                    child: Text(
                                      '${point.shortCode}: ${point.label}',
                                      maxLines: 1,
                                      overflow: TextOverflow.ellipsis,
                                      style: const TextStyle(
                                        color: Colors.white,
                                        fontSize: 10,
                                        fontWeight: FontWeight.bold,
                                      ),
                                    ),
                                  ),
                                  Icon(
                                    Icons.location_on,
                                    color: isStart
                                        ? Colors.green.shade700
                                        : Colors.red.shade600,
                                    size: 36,
                                  ),
                                ],
                              ),
                            ),
                          );
                        }).toList(),
                      ),
                    ],
                  ),
                  Positioned(
                    top: 10,
                    left: 10,
                    right: 10,
                    child: Container(
                      padding: const EdgeInsets.symmetric(
                          horizontal: 12, vertical: 8),
                      decoration: BoxDecoration(
                        color: Colors.black87,
                        borderRadius: BorderRadius.circular(8),
                      ),
                      child: Row(
                        mainAxisAlignment: MainAxisAlignment.spaceBetween,
                        children: [
                          Expanded(
                            child: Text(
                              'Center Lat: ${_currentCenter.latitude.toStringAsFixed(5)} | Lng: ${_currentCenter.longitude.toStringAsFixed(5)}',
                              style: const TextStyle(
                                color: Colors.white,
                                fontSize: 12,
                                fontFamily: 'monospace',
                              ),
                            ),
                          ),
                          const Text(
                            'Tap peta untuk +titik',
                            style: TextStyle(
                                color: Colors.amberAccent, fontSize: 11),
                          ),
                        ],
                      ),
                    ),
                  ),
                ],
              ),
            ),
            Card(
              margin: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              elevation: 2,
              child: Padding(
                padding: const EdgeInsets.all(12),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: [
                        const Text(
                          'Input Titik Koordinat Tujuan',
                          style: TextStyle(
                              fontWeight: FontWeight.bold, fontSize: 15),
                        ),
                        TextButton.icon(
                          onPressed: _loadSamplePreset,
                          icon: const Icon(Icons.playlist_add_check,
                              size: 18),
                          label: const Text('Isi 4 Contoh Titik',
                              style: TextStyle(fontSize: 12)),
                        ),
                      ],
                    ),
                    const SizedBox(height: 8),
                    Row(
                      children: [
                        Expanded(
                          child: TextField(
                            controller: _latController,
                            keyboardType:
                                const TextInputType.numberWithOptions(
                                    decimal: true, signed: true),
                            decoration: const InputDecoration(
                              labelText: 'Latitude',
                              hintText: '-6.2088',
                              border: OutlineInputBorder(),
                              isDense: true,
                            ),
                          ),
                        ),
                        const SizedBox(width: 8),
                        Expanded(
                          child: TextField(
                            controller: _lngController,
                            keyboardType:
                                const TextInputType.numberWithOptions(
                                    decimal: true, signed: true),
                            decoration: const InputDecoration(
                              labelText: 'Longitude',
                              hintText: '106.8456',
                              border: OutlineInputBorder(),
                              isDense: true,
                            ),
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 8),
                    Row(
                      children: [
                        Expanded(
                          child: TextField(
                            controller: _labelController,
                            decoration: const InputDecoration(
                              labelText:
                                  'Keterangan Lokasi (misal: Gudang Sentral)',
                              border: OutlineInputBorder(),
                              isDense: true,
                            ),
                          ),
                        ),
                        const SizedBox(width: 8),
                        ElevatedButton.icon(
                          onPressed: _submitFormInput,
                          icon: const Icon(Icons.add_location_alt),
                          label: const Text('Tambah'),
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.indigo,
                            foregroundColor: Colors.white,
                            padding: const EdgeInsets.symmetric(
                                horizontal: 14, vertical: 12),
                          ),
                        ),
                      ],
                    ),
                    const Divider(height: 20),
                    Text(
                      'Daftar Titik Koordinat Terdaftar (${_points.length} Titik):',
                      style: const TextStyle(
                          fontWeight: FontWeight.bold, fontSize: 13),
                    ),
                    const SizedBox(height: 6),
                    _points.isEmpty
                        ? const Padding(
                            padding: EdgeInsets.symmetric(vertical: 8),
                            child: Text(
                              'Belum ada titik. Tambahkan lewat form di atas atau ketuk langsung pada peta.',
                              style: TextStyle(
                                  color: Colors.grey, fontSize: 12),
                            ),
                          )
                        : ListView.separated(
                            shrinkWrap: true,
                            physics: const NeverScrollableScrollPhysics(),
                            itemCount: _points.length,
                            separatorBuilder: (_, __) =>
                                const Divider(height: 1),
                            itemBuilder: (context, idx) {
                              final item = _points[idx];
                              final bool isStart = item.index == 1;
                              return ListTile(
                                dense: true,
                                contentPadding: EdgeInsets.zero,
                                leading: CircleAvatar(
                                  radius: 14,
                                  backgroundColor: isStart
                                      ? Colors.green.shade700
                                      : Colors.red.shade600,
                                  child: Text(
                                    item.shortCode,
                                    style: const TextStyle(
                                        color: Colors.white,
                                        fontSize: 11,
                                        fontWeight: FontWeight.bold),
                                  ),
                                ),
                                title: Text(
                                  '${item.label}${isStart ? " (Titik Awal)" : ""}',
                                  style: const TextStyle(
                                      fontWeight: FontWeight.w600),
                                ),
                                subtitle: Text(
                                  'Lat: ${item.latitude.toStringAsFixed(5)}, Lng: ${item.longitude.toStringAsFixed(5)}',
                                  style: const TextStyle(fontSize: 11),
                                ),
                                trailing: IconButton(
                                  icon: const Icon(Icons.delete_outline,
                                      color: Colors.red, size: 20),
                                  onPressed: () => _removePoint(idx),
                                  tooltip: 'Hapus Titik',
                                ),
                              );
                            },
                          ),
                  ],
                ),
              ),
            ),
            Padding(
              padding:
                  const EdgeInsets.symmetric(horizontal: 12, vertical: 4),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  const Text(
                    'TOMBOL OPERASI RUTE POLYLINE:',
                    style: TextStyle(
                        fontWeight: FontWeight.bold, fontSize: 13),
                  ),
                  const SizedBox(height: 6),
                  ElevatedButton.icon(
                    onPressed: _runSequentialRoute,
                    icon: const Icon(Icons.timeline),
                    label:
                        const Text('1. Rute Sesuai Urutan Input (Sekuensial)'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.orange.shade700,
                      foregroundColor: Colors.white,
                    ),
                  ),
                  const SizedBox(height: 6),
                  ElevatedButton.icon(
                    onPressed: _runDijkstraRoute,
                    icon: const Icon(Icons.alt_route),
                    label: const Text('2. Rute Terpendek Dijkstra'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.blue.shade800,
                      foregroundColor: Colors.white,
                    ),
                  ),
                  const SizedBox(height: 6),
                  ElevatedButton.icon(
                    onPressed: _runAlternativeRoute,
                    icon: const Icon(Icons.auto_graph),
                    label:
                        const Text('3. Rute Terpendek A* Search (Alternatif)'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.purple.shade700,
                      foregroundColor: Colors.white,
                    ),
                  ),
                  const SizedBox(height: 6),
                  OutlinedButton.icon(
                    onPressed: () => setState(() => _clearRouteResults()),
                    icon: const Icon(Icons.layers_clear),
                    label: const Text('Reset Garis Rute'),
                  ),
                ],
              ),
            ),
            Builder(builder: (context) {
              final double? seqDist = _sequentialResult?.totalDistanceKm;
              final double? dijDist = _dijkstraResult?.totalDistanceKm;
              final double? altDist = _alternativeResult?.totalDistanceKm;

              double? bestOptimizedDist;
              String bestAlgoName = '-';
              if (dijDist != null && altDist != null) {
                if (altDist <= dijDist) {
                  bestOptimizedDist = altDist;
                  bestAlgoName = 'A* Search';
                } else {
                  bestOptimizedDist = dijDist;
                  bestAlgoName = 'Dijkstra';
                }
              } else if (dijDist != null) {
                bestOptimizedDist = dijDist;
                bestAlgoName = 'Dijkstra';
              } else if (altDist != null) {
                bestOptimizedDist = altDist;
                bestAlgoName = 'A* Search';
              }

              double savedKm = 0.0;
              double savedPercent = 0.0;
              if (seqDist != null && seqDist > 0 && bestOptimizedDist != null) {
                savedKm = (seqDist - bestOptimizedDist)
                    .clamp(0.0, double.infinity);
                savedPercent = (savedKm / seqDist) * 100.0;
              }

              return Card(
                margin: const EdgeInsets.fromLTRB(12, 6, 12, 20),
                elevation: 3,
                color: Colors.indigo.shade50,
                child: Padding(
                  padding: const EdgeInsets.all(14),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Row(
                        mainAxisAlignment: MainAxisAlignment.spaceBetween,
                        children: [
                          const Text(
                            'PANEL RINGKASAN & KOMPARASI RUTE',
                            style: TextStyle(
                              fontWeight: FontWeight.bold,
                              fontSize: 14,
                              color: Colors.indigo,
                            ),
                          ),
                          Chip(
                            label: Text('Total Titik: ${_points.length}'),
                            backgroundColor: Colors.indigo.shade100,
                            labelStyle: const TextStyle(
                                fontSize: 12, fontWeight: FontWeight.bold),
                            visualDensity: VisualDensity.compact,
                          ),
                        ],
                      ),
                      if (_activeRoute != null) ...[
                        const SizedBox(height: 6),
                        Container(
                          width: double.infinity,
                          padding: const EdgeInsets.all(8),
                          decoration: BoxDecoration(
                            color: _activeRoute!.lineColor
                                .withValues(alpha: 0.12),
                            borderRadius: BorderRadius.circular(6),
                            border:
                                Border.all(color: _activeRoute!.lineColor),
                          ),
                          child: Text(
                            'Rute Aktif di Peta: ${_activeRoute!.algorithmName}\nUrutan: ${_activeRoute!.sequenceText}',
                            style: TextStyle(
                              fontSize: 12,
                              fontWeight: FontWeight.w600,
                              color: _activeRoute!.lineColor,
                            ),
                          ),
                        ),
                      ],
                      const Divider(height: 18),
                      _buildSummaryRow(
                        title: '1. Rute Sekuensial (Urutan Input)',
                        result: _sequentialResult,
                        badgeColor: Colors.orange.shade700,
                      ),
                      const SizedBox(height: 8),
                      _buildSummaryRow(
                        title: '2. Rute Terpendek Dijkstra',
                        result: _dijkstraResult,
                        badgeColor: Colors.blue.shade800,
                      ),
                      const SizedBox(height: 8),
                      _buildSummaryRow(
                        title: '3. Rute A* Search (Alternatif)',
                        result: _alternativeResult,
                        badgeColor: Colors.purple.shade700,
                      ),
                      const Divider(height: 20),
                      Container(
                        width: double.infinity,
                        padding: const EdgeInsets.all(12),
                        decoration: BoxDecoration(
                          color: Colors.green.shade800,
                          borderRadius: BorderRadius.circular(8),
                        ),
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            const Text(
                              'ANALISIS EFISIENSI PENGHEMATAN JARAK',
                              style: TextStyle(
                                color: Colors.white70,
                                fontSize: 11,
                                fontWeight: FontWeight.bold,
                              ),
                            ),
                            const SizedBox(height: 4),
                            Text(
                              (seqDist != null && bestOptimizedDist != null)
                                  ? 'Penghematan: ${savedKm.toStringAsFixed(2)} km (${savedPercent.toStringAsFixed(1)}%) — Terbaik: $bestAlgoName'
                                  : 'Jalankan Rute Sekuensial dan Rute Optimasi untuk melihat persentase penghematan.',
                              style: const TextStyle(
                                color: Colors.white,
                                fontSize: 13,
                                fontWeight: FontWeight.bold,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ],
                  ),
                ),
              );
            }),
          ],
        ),
      ),
    );
  }

  Widget _buildSummaryRow({
    required String title,
    required RouteResult? result,
    required Color badgeColor,
  }) {
    return Row(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Container(
          width: 10,
          height: 10,
          margin: const EdgeInsets.only(top: 4, right: 8),
          decoration: BoxDecoration(color: badgeColor, shape: BoxShape.circle),
        ),
        Expanded(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                title,
                style: const TextStyle(
                    fontWeight: FontWeight.bold, fontSize: 13),
              ),
              Text(
                result == null
                    ? 'Belum dikalkulasi'
                    : 'Jarak: ${result.totalDistanceKm.toStringAsFixed(2)} km | Urutan: ${result.compactSequenceText}',
                style: TextStyle(
                  fontSize: 12,
                  color: result == null ? Colors.grey : Colors.black87,
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }
}