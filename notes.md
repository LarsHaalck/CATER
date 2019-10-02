> keyFrames = imageContainer.getKeyFrames();
> cache = imageContainer.getCache(keyFrames, chunkSize));
> for chunkNum in cache.getNumChunks(): // ceil(N / K)
>     chunk = cache.retrieveChunk(chunkNum):
>     start, end = chunk.ids
>     for elem in chunk:
>         // handle elem

Images (cv::Mat)
-------------
- getKeyFrames
- markAsKeyFrame
- Chunking (non redundant) for feature/descriptor extraction
- gray vs color?


Descriptors (per image: std::vector<cv::Mat>)
-------------
- only needed for putative matching
- chunking (redundant?) for putative matching?
- best case:  k Cache --> k+1 Elems
- worst case: k Cache --> 2k Elems
- cache allocation size? (1-2)k?


Features (per image: std::vector<cv::KeyPoint>)
-------------
- needed multiple times
- need to mark "good" features based on heuristics --> store in std::vector<std::size_t>?
- same as descriptors otherwise

Matches (possibly per pair of images: std::unorded_map<std::pair<std::size_t, std::size_t>, MatchType>)
---------------------
- MatchType --> std::vector<cv::DMatch> vs much smaller: std::vector<std::pair<std::size_t, std::size_t>>

Transformations (possibly per pair of images, similar to Matches)
----------------------------------------------
- Type: Homography, Affinity, Similarity, Isometry
- local transformation (i,j) vs global Transformation T_i --> encode as global? or as  (0, j) ?
- maybe distinguishing works better here?
- always use correct direction forward vs backward, what is needed more?



# CHUNKING:
MISC:
-----------
N steps (image or pairs), k Cache
ceil(N / k) = 1 + ((N - 1) / k)
floor(N / k) = N / k

numChunks = ceil(N / K)
for i in range(numChunks):
    chunk = getChunk(i)

    @parallel
    for elem in chunk:
        doStuff()

Chunker<Class>(N, k)
Chunker<Class>(const std::vector<std::size_t>& keyFrames, k)

chunker.getNumChunks()
getChunkSize(i)
retrieveChunk() --> should have no cost if everything is in memory (i.e. small videos)


CacheMember in every class?
How to control cache? per call basis? globally?
flag to disable function?


cache member vs inheritance vs something else

Cache Class: return as unique_ptr from reader class, should no survive original class
------------------
getNumChunks ziemlich easy
getChunk --> return value?


Cache Policies:
--------------------
1: chunk for parallel for
2: rechnen
3: remove chunk
goto 1
#################################
1: chunk for parallel for
2: rechnen
3: add to chunk reuse if possible
goto 1
#################################
1: parallel rechnen + add to cache if new, reuse if exists


cache redudant vs non-redundant:
e.g. used elems
0
0
1
1
2
2

store 3 elems or 6?


Cache Class:
--------------------
constructor easy peasy






