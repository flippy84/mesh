#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Recast.h"
#include "MeshLoaderObj.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "ChunkyTriMesh.h"
#include <Windows.h>

/*
adt > local coordinates 0 - 533 max

buildmesh
set mesh orig, build detailmesh -> detailmesh gets world coords

set nav orig, build navmesh -> navmesh gets world coords, detailmesh not changed
*/

unsigned char* buildTileMesh(rcMeshLoaderObj* m_mesh, int tx, int ty, float* bmin, float *bmax, int& dataSize);
void saveAll(const char* path, const dtNavMesh* mesh);

class BuildContext : public rcContext
{
    virtual void doLog(const rcLogCategory /*category*/, const char* /*msg*/, const int /*len*/);
};

void BuildContext::doLog(const rcLogCategory category, const char* msg, const int len)
{
	if(category == RC_LOG_ERROR)
		puts(msg);
}

struct thread_params
{
	rcMeshLoaderObj* mesh;
	int tx, ty;
	float bmin[3];
	float bmax[3];
	int size;
	unsigned char* data;
};

DWORD WINAPI WorkerThread(_In_ LPVOID lpParam)
{
	thread_params* t_param = (thread_params*)lpParam;

	t_param->data = buildTileMesh(t_param->mesh, t_param->tx, t_param->ty, t_param->bmin, t_param->bmax, t_param->size);

	return 0;
}

VOID CALLBACK WorkerCallback(PTP_CALLBACK_INSTANCE instance, PVOID context, PTP_WORK work)
{
	thread_params* t_param = (thread_params*)context;

	t_param->data = buildTileMesh(t_param->mesh, t_param->tx, t_param->ty, t_param->bmin, t_param->bmax, t_param->size);

	return;
}

VOID WINAPI CleanupGroupMembers(PTP_CLEANUP_GROUP ptpcg, BOOL fCancelPendingCallbacks, PVOID pvCleanupContext)
{
	return;
}

rcConfig m_cfg;

int main(int argc, char* argv[])
{
    double TileSize = 1600.0 / 12;
    int VoxelCount = 1000;
    float m_cellSize = (float)(TileSize / VoxelCount);
    float m_cellHeight = (float)0.1;

    memset(&m_cfg, 0, sizeof(m_cfg));
    m_cfg.cs = (float)(TileSize / VoxelCount);
    m_cfg.ch = (float)0.1;
    m_cfg.walkableSlopeAngle = 45;
	m_cfg.walkableHeight = (int)ceil(2.1 / m_cfg.ch);
	m_cfg.walkableClimb = (int)floor(1.0 / m_cfg.ch);
	m_cfg.walkableRadius = (int)ceil(0.3 / m_cfg.cs);
    m_cfg.maxEdgeLen = (int)(12 / m_cfg.cs);
	m_cfg.maxSimplificationError = (float)1.3;
	m_cfg.minRegionArea = rcSqr(8);		// Note: area = size*size
	m_cfg.mergeRegionArea = rcSqr(20);	// Note: area = size*size
	m_cfg.maxVertsPerPoly = 6;

	float m_detailSampleDist = 6;
    m_cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;

    float m_detailSampleMaxError = 1;
	m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
	//m_cfg.detailSampleMaxError = 0;
    m_cfg.tileSize = VoxelCount;
    m_cfg.borderSize = m_cfg.walkableRadius + 3; // Reserve enough padding.
	m_cfg.width = m_cfg.tileSize + m_cfg.borderSize*2;
	m_cfg.height = m_cfg.tileSize + m_cfg.borderSize*2;

    rcMeshLoaderObj* m_mesh = new rcMeshLoaderObj();
    if(!m_mesh->load(argv[1]))
    {
        printf("Can't open %s\n", argv[1]);
        return 0;
    }

    float bmin[3];
    float bmax[3];
    rcCalcBounds(m_mesh->getVerts(), m_mesh->getVertCount(), bmin, bmax);

    int mesh_x, mesh_y;
    sscanf_s(argv[1], "%*[^_]_%d_%d.obj", &mesh_x, &mesh_y);

    //Right handed coordinates

    bmin[0] = (float)((mesh_x - 32) * 1600.0 / 3);
    bmax[0] = (float)(bmin[0] + 1600.0 / 3);
    bmin[2] = (float)((mesh_y - 32) * 1600.0 / 3);
    bmax[2] = bmin[2] + 1600.0f / 3;

    rcVcopy(m_cfg.bmin, bmin);
    rcVcopy(m_cfg.bmax, bmax);

	float m_tileBmin[3];
	float m_tileBmax[3];

    int w, h;
    rcCalcGridSize(bmin, bmax, m_cfg.cs, &w, &h);
    const int ts = m_cfg.tileSize;
	const int tw = (w + ts-1) / ts;
	const int th = (h + ts-1) / ts;
    const double tcs = m_cfg.tileSize*m_cellSize;
    //const float tcs = 400.0 / 3;
    //const float tcs = 400.0 / 3;

    dtNavMeshParams params;
    rcVcopy(params.orig, bmin);
    params.tileWidth = m_cfg.tileSize*m_cellSize;
	params.tileHeight = m_cfg.tileSize*m_cellSize;
	params.maxTiles = 1 << 16;
	params.maxPolys = 1 << 16;

    dtNavMesh* m_navMesh = new dtNavMesh();
    m_navMesh->init(&params);

	thread_params t_params[16];
	int size[16];

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			m_tileBmin[0] = (float)(bmin[0] + x*tcs);
			m_tileBmin[1] = bmin[1];
			m_tileBmin[2] = (float)(bmin[2] + y*tcs);
			
			m_tileBmax[0] = (float)(bmin[0] + (x+1)*tcs);
			m_tileBmax[1] = bmax[1];
			m_tileBmax[2] = (float)(bmin[2] + (y+1)*tcs);


            /*m_tileBmin[0] = (((mesh_x - 32) * 4 + x) * 1600) / 12.0;
			m_tileBmin[1] = bmin[1];
			m_tileBmin[2] = (((mesh_y - 32) * 4 + y) * 1600) / 12.0;
			
			m_tileBmax[0] = (((mesh_x - 32) * 4 + x + 1) * 1600) / 12.0;
			m_tileBmax[1] = bmax[1];
			m_tileBmax[2] = (((mesh_y - 32) * 4 + y + 1) * 1600) / 12.0;*/

			//m_navMesh->calcTileLoc(m_tileBmin, &tx, &ty);

            m_tileBmin[0] -= m_cfg.borderSize*m_cellSize;
            m_tileBmin[2] -= m_cfg.borderSize*m_cellSize;

            m_tileBmax[0] += m_cfg.borderSize*m_cellSize;
            m_tileBmax[2] += m_cfg.borderSize*m_cellSize;
			
			int dataSize = 0;

			t_params[y*4+x].mesh = m_mesh;
			t_params[y*4+x].tx = (mesh_x - 32) * 4 + x;
			t_params[y*4+x].ty = (mesh_y - 32) * 4 + y;
			rcVcopy(t_params[y*4+x].bmin, m_tileBmin);
			rcVcopy(t_params[y*4+x].bmax, m_tileBmax);
			t_params[y*4+x].size = size[y*4+x];

			//threads[y*4+x] = CreateThread(0, 0, WorkerThread, &t_params[y*4+x], CREATE_SUSPENDED, 0);

			//unsigned char* data = buildTileMesh(m_mesh, (mesh_x - 32) * 4 + x, (mesh_y - 32) * 4 + y, m_tileBmin, m_tileBmax, dataSize);

			//unsigned char* data = buildTileMesh(m_mesh, tx, ty, m_tileBmin, m_tileBmax, dataSize);
			/*if (data)
			{
				dtStatus status = m_navMesh->addTile(data,dataSize,DT_TILE_FREE_DATA,0,0);
				if (dtStatusFailed(status))
					dtFree(data);
			}*/
            //printf("Tile (%d, %d) done\n", x, y);
		}
	}

	PTP_POOL pool = CreateThreadpool(0);
	SetThreadpoolThreadMaximum(pool, 1);
	SetThreadpoolThreadMinimum(pool, 4);

	TP_CALLBACK_ENVIRON pcbe;
	InitializeThreadpoolEnvironment(&pcbe);
	SetThreadpoolCallbackPool(&pcbe, pool);

	PTP_CLEANUP_GROUP c_group = CreateThreadpoolCleanupGroup();
	SetThreadpoolCallbackCleanupGroup(&pcbe, c_group, 0);
	PTP_WORK work;

	for(int i = 0; i < 16; i++)
	{
		work = CreateThreadpoolWork(WorkerCallback, &t_params[i], &pcbe);
		SubmitThreadpoolWork(work);
	}

	WaitForThreadpoolWorkCallbacks(work, false);

	CloseThreadpoolCleanupGroupMembers(c_group, false, CleanupGroupMembers);
	CloseThreadpoolCleanupGroup(c_group);
	CloseThreadpool(pool);

	for(int i = 0; i < 16; i++)
	{
		dtStatus status = m_navMesh->addTile(t_params[i].data, t_params[i].size, DT_TILE_FREE_DATA, 0, 0);
		if(dtStatusFailed(status))
		{
			dtFree(t_params[i].data);
			puts("addTile failed");
		}
	}

    char file[512];
    memset(file, 0, sizeof(file));
    strncpy_s(file, argv[1], strlen(argv[1]) - 3);
    strcat_s(file, "bin");
    saveAll(file, m_navMesh);
    //saveAll("all_tiles_navmesh.bin", m_navMesh);
    printf("Saved %s\n", file);

	return 0;
}

enum SamplePolyAreas
{
	SAMPLE_POLYAREA_GROUND,
	SAMPLE_POLYAREA_WATER,
	SAMPLE_POLYAREA_ROAD,
	SAMPLE_POLYAREA_DOOR,
	SAMPLE_POLYAREA_GRASS,
	SAMPLE_POLYAREA_JUMP,
};

enum SamplePolyFlags
{
	SAMPLE_POLYFLAGS_WALK		= 0x01,		// Ability to walk (ground, grass, road)
	SAMPLE_POLYFLAGS_SWIM		= 0x02,		// Ability to swim (water).
	SAMPLE_POLYFLAGS_DOOR		= 0x04,		// Ability to move through doors.
	SAMPLE_POLYFLAGS_JUMP		= 0x08,		// Ability to jump.
	SAMPLE_POLYFLAGS_DISABLED	= 0x10,		// Disabled polygon
	SAMPLE_POLYFLAGS_ALL		= 0xffff	// All abilities.
};

unsigned char* buildTileMesh(rcMeshLoaderObj* m_mesh, int tx, int ty, float* tmin, float *tmax, int& dataSize)
{
    rcContext* m_ctx = new BuildContext();
    rcHeightfield* m_solid = rcAllocHeightfield();

    int w, h;
    rcCalcGridSize(tmin, tmax, m_cfg.cs, &w, &h);

    unsigned char* m_triareas = new unsigned char[m_mesh->getTriCount()];

    if(!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height, tmin, tmax, m_cfg.cs, m_cfg.ch))
    {
        puts("rcCreateHeightfield");
        return 0;
    }

    rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle, m_mesh->getVerts(), m_mesh->getVertCount(), m_mesh->getTris(), m_mesh->getTriCount(), m_triareas);
    rcRasterizeTriangles(m_ctx, m_mesh->getVerts(), m_mesh->getVertCount(), m_mesh->getTris(), m_triareas, m_mesh->getTriCount(), *m_solid, m_cfg.walkableClimb);

    delete [] m_triareas;

    rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);
	rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);
	rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);

    rcCompactHeightfield* m_chf = rcAllocCompactHeightfield();
    if(!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
    {
        puts("rcBuildCompactHeightfield");
    }
    rcFreeHeightField(m_solid);

    if(!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
    {
        puts("rcErodeWalkableArea");
    }

    if(!rcBuildDistanceField(m_ctx, *m_chf))
    {
        puts("rcBuildDistanceField");
    }
    rcBuildRegions(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea);

    rcContourSet* m_cset = rcAllocContourSet();
    if(!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
    {
        puts("rcBuildContours");
    }

    rcPolyMesh* m_pmesh = rcAllocPolyMesh();
    if(!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
    {
        puts("rcBuildPolyMesh");
    }

    rcPolyMeshDetail* m_dmesh = rcAllocPolyMeshDetail();
    if(!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf, m_cfg.detailSampleDist, m_cfg.detailSampleMaxError, *m_dmesh))
    {
        puts("rcBuildPolyMeshDetail");
    }

    for(int i = 0; i < m_pmesh->npolys; i++)
    {
        if(m_pmesh->areas[i] == RC_WALKABLE_AREA)
        {
            m_pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;
            m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
        }
		/*else
		{
			m_pmesh->areas[i] = RC_NULL_AREA;
			m_pmesh->flags[i] = SAMPLE_POLYFLAGS_DISABLED;
		}*/
    }

    rcFreeCompactHeightfield(m_chf);
    rcFreeContourSet(m_cset);

    dtNavMeshCreateParams params;
    memset(&params, 0, sizeof(params));
    params.verts = m_pmesh->verts;
    params.vertCount = m_pmesh->nverts;
    params.polys = m_pmesh->polys;
    params.polyAreas = m_pmesh->areas;
    params.polyFlags = m_pmesh->flags;
    params.polyCount = m_pmesh->npolys;
    params.nvp = m_pmesh->nvp;
    params.detailMeshes = m_dmesh->meshes;
    params.detailVerts = m_dmesh->verts;
    params.detailVertsCount = m_dmesh->nverts;
    params.detailTris = m_dmesh->tris;
    params.detailTriCount = m_dmesh->ntris;
    /*params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
    params.offMeshConRad = m_geom->getOffMeshConnectionRads();
    params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
    params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
    params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
    params.offMeshConUserID = m_geom->getOffMeshConnectionId();
    params.offMeshConCount = m_geom->getOffMeshConnectionCount();*/
    params.walkableHeight = 2.1f;
    params.walkableRadius = 0.3f;
    params.walkableClimb = 1.0f;

    rcVcopy(params.bmin, m_pmesh->bmin);
    rcVcopy(params.bmax, m_pmesh->bmax);
    params.tileLayer = 0;
    params.tileX = tx;
    params.tileY = ty;

    params.cs = m_cfg.cs;
    params.ch = m_cfg.ch;
    params.buildBvTree = true;

    unsigned char* navData;
    int navDataSize;

    if(!dtCreateNavMeshData(&params, &navData, &navDataSize))
    {
        puts("dtCreateNavMeshData");
    }

    rcFreePolyMeshDetail(m_dmesh);

    dataSize = navDataSize;
	printf("Tile (%d, %d) done\n", tx, ty);
    return navData;
}

static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

void saveAll(const char* path, const dtNavMesh* mesh)
{
	if (!mesh) return;
	
	FILE* fp;
	fopen_s(&fp, path, "wb");
	if (!fp)
		return;
	
	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}

	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);
}