using UnityEngine;
using Unity.Mathematics;

public enum CollisionType
{
    SphereSphere , CapsuleLockCapsuleLock, CaspuleCapsule, BoxBox3D, SphereCapsule, CapsuleBox3D, SphereBox
}

public class CollisionTester : MonoBehaviour
{
    public CollisionType collisionType;

    public float3 posA;
    public float3 posB;

    public float3 rotA;
    public float3 rotB;

    public float3 halfSizeA;
    public float3 halfSizeB;

    private GameObject shapeA;
    private GameObject shapeB;

    private CollisionType lastCollisionType;

    private void Start()
    {
        SetupScene();
    }
    private void Update()
    {
        if ( collisionType != lastCollisionType )
        {
            lastCollisionType = collisionType;
            SetupScene();
        }

        switch ( collisionType )
        {
            case CollisionType.SphereSphere:
            HandleSpheres();
            break;
            case CollisionType.SphereCapsule:
            HandleSphereCapsule();
            break;
            case CollisionType.SphereBox:
            HandleSphereBox();
            break;
            case CollisionType.CapsuleLockCapsuleLock:
            HandleCapsulesLock();
            break;
            case CollisionType.CaspuleCapsule:
            HandleCapsuleCapsule();
            break;
            case CollisionType.BoxBox3D:
            HandleBoxBox3D();
            break;
            default:
            Debug.Log( "No CollisionType Selected" );
            break;
        }
    }

    private void SetupScene()
    {
        ResetTransform();

        if ( shapeA )
            Destroy( shapeA );
        if ( shapeB )
            Destroy( shapeB );

        switch ( collisionType )
        {
            case CollisionType.SphereSphere:
            SetupSpheres();
            break;
            case CollisionType.SphereCapsule:
            SetupSphereCapsule();
            break;
            case CollisionType.SphereBox:
            SetupSphereBox();
            break;
            case CollisionType.CapsuleLockCapsuleLock:
            SetupLockedCapsule();
            break;
            case CollisionType.CaspuleCapsule:
            SetupCapsuleCapsule();
            break;
            case CollisionType.BoxBox3D:
            SetupBoxBox();
            break;
            default:
            Debug.Log( "No CollisionType Selected" );
            break;
        }
    }
    private void SetupSpheres()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Sphere );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Sphere );
    }
    private void SetupLockedCapsule()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Capsule );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Capsule );
    }
    private void SetupCapsuleCapsule()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Capsule );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Capsule );
    }
    private void SetupBoxBox()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Cube );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Cube );
    }
    private void SetupSphereCapsule()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Sphere );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Capsule );
    }
    private void SetupSphereBox()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Sphere );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Cube );
    }
    private void ResetTransform()
    {
        posA = float3.zero;
        posB = new float3( 5 , 0 , 0 );

        rotA = float3.zero;
        rotB = float3.zero;

        halfSizeA = new float3( 1 , 1 , 1 );
        halfSizeB = new float3( 1 , 1 , 1 );
    }

    // SPHERE SPHERE
    private void HandleSpheres()
    {
        CollisionSphere( ref posA , ref posB , halfSizeA.x , halfSizeB.x );
        DrawSpheres();
    }
    private void CollisionSphere( ref float3 _posA , ref float3 _posB , float radiusA , float radiusB )
    {
        float distance = math.distance( _posA , _posB );
        float sumRadii = radiusA + radiusB;

        if ( distance <= sumRadii )
        {
            float3 displacement = CalculateSphereCollisionDisplacement( _posA , _posB , radiusA , radiusB , distance );

            _posA -= displacement;
            _posB += displacement;
        }
    }

    // CAPSULELOCK CAPSULELOCK
    private void HandleCapsulesLock()
    {
        CollisionCapsuleLock( ref posA , ref posB , halfSizeA.x , halfSizeB.x , halfSizeA.y , halfSizeB.y );
        DrawCapsuleLock();
    }
    private void CollisionCapsuleLock( ref float3 _posA , ref float3 _posB , float radiusA , float radiusB , float heightA, float heightB )
    {
        if ( _posA.y + heightA >= _posB.y - heightB )
        {
            CollisionSphere( ref _posA , ref _posB , radiusA , radiusB );
            /*float distance = math.distance( _posA , _posB );
            float sumRadii = radiusA + radiusB;

            if ( distance <= sumRadii )
            {
                float3 displacement = CalculateSphereCollisionDisplacement( _posA , _posB , radiusA , radiusB , distance );

                _posA -= displacement;
                _posB += displacement;
            }*/
        }
    }

    // CAPSULE CAPSULE
    private struct CapsulePoints
    {
        public float3 tipPoint;
        public float3 basePoint;
    }
    private void HandleCapsuleCapsule()
    {
        CollisionCapsuleCapsule();
        DrawCapsules();
    }
    private void CollisionCapsuleCapsule()
    {
        CapsulePoints pointsA = GetCapsuleCollider( posA , rotA , halfSizeA.y * 2 );
        CapsulePoints pointsB = GetCapsuleCollider( posB , rotB , halfSizeB.y * 2 );

        CapsulePoints spheresA = GetCapsuleSpheres( pointsA.tipPoint , pointsA.basePoint , halfSizeA.x );
        CapsulePoints spheresB = GetCapsuleSpheres( pointsB.tipPoint , pointsB.basePoint , halfSizeB.x );

        float3 v0 = spheresB.basePoint - spheresA.basePoint;
        float3 v1 = spheresB.tipPoint - spheresA.basePoint;
        float3 v2 = spheresB.basePoint - spheresA.tipPoint;
        float3 v3 = spheresA.tipPoint - spheresA.tipPoint;

        float d0 = math.dot( v0 , v0 );
        float d1 = math.dot( v1 , v1 );
        float d2 = math.dot( v2 , v2 );
        float d3 = math.dot( v3 , v3 );

        float3 closestPointA = math.select( spheresA.basePoint , spheresA.tipPoint , d2 < d0 || d2 < d1 || d3 < d0 || d3 < d1 );
        float3 closestPointB = ClosestPointOnLineSegement( spheresB.basePoint , spheresB.tipPoint , closestPointA );
        closestPointA = ClosestPointOnLineSegement( spheresA.basePoint , spheresA.tipPoint , closestPointB );

        float distance = math.distance( closestPointA , closestPointB );
        float penetrationDepth = halfSizeA.x + halfSizeB.x - distance;
        bool intersects = penetrationDepth > 0;

        if ( intersects )
        {
            ResolveCapsuleCollision( ref posA , ref posB , closestPointA , closestPointB , halfSizeA.x , halfSizeB.x , distance );
        }
    }
    private void ResolveCapsuleCollision( ref float3 originA , ref float3 originB , float3 pointA , float3 pointB , float radiusA , float radiusB , float distance )
    {
        float3 displacement = CalculateSphereCollisionDisplacement( pointA , pointB , radiusA , radiusB , distance );

        originA -= displacement;
        originB += displacement;
    }
    private CapsulePoints GetCapsuleCollider( float3 position , float3 rotation , float length )
    {
        float3 forward = math.forward( quaternion.EulerXYZ( rotation ) );
        float3 axisLine = forward * length;
        float3 tipPoint = position + axisLine;
        float3 basePoint = position - axisLine;

        return new CapsulePoints
        {
            tipPoint = tipPoint ,
            basePoint = basePoint
        };
    }
    private CapsulePoints GetCapsuleSpheres( float3 tipPoint , float3 basePoint , float radius )
    {
        float3 normal = math.normalize( tipPoint -basePoint );
        float3 lineEndOffset = normal * radius;
        float3 baseSphere = basePoint + lineEndOffset;
        float3 tipSphere = tipPoint - lineEndOffset;

        return new CapsulePoints
        {
            basePoint = baseSphere ,
            tipPoint = tipSphere
        };
    }

    // BOX BOX
    private void HandleBoxBox3D()
    {
        float3[] vertsA = GetRotatedVerticesOfBox( posA , halfSizeA , rotA );
        float3[] vertsB = GetRotatedVerticesOfBox( posB , halfSizeB , rotB );

        float3[] axesA = GetProjectionAxesOfBox( vertsA );
        float3[] axesB = GetProjectionAxesOfBox( vertsB );

        float3[] allAxes = new float3[ 15 ];
        allAxes[ 0 ] = axesA[ 0 ];
        allAxes[ 1 ] = axesA[ 1 ];
        allAxes[ 2 ] = axesA[ 2 ];
        allAxes[ 3 ] = axesB[ 0 ];
        allAxes[ 4 ] = axesB[ 1 ];
        allAxes[ 5 ] = axesB[ 2 ];
        allAxes[ 6 ] = math.cross( axesA[ 0 ] , axesB[ 0 ] );
        allAxes[ 7 ] = math.cross( axesA[ 0 ] , axesB[ 1 ] );
        allAxes[ 8 ] = math.cross( axesA[ 0 ] , axesB[ 2 ] );
        allAxes[ 9 ] = math.cross( axesA[ 1 ] , axesB[ 0 ] );
        allAxes[ 10 ] = math.cross( axesA[ 1 ] , axesB[ 1 ] );
        allAxes[ 11 ] = math.cross( axesA[ 1 ] , axesB[ 2 ] );
        allAxes[ 12 ] = math.cross( axesA[ 2 ] , axesB[ 0 ] );
        allAxes[ 13 ] = math.cross( axesA[ 2 ] , axesB[ 1 ] );
        allAxes[ 14 ] = math.cross( axesA[ 2 ] , axesB[ 2 ] );

        CollisionBoxBox( allAxes , vertsA , vertsB , ref posA , ref posB );
        DrawBoxes();
    }
    private void CollisionBoxBox( float3[] allAxes , float3[] vertsA , float3[] vertsB , ref float3 originA , ref float3 originB )
    {
        float minOverlap = float.PositiveInfinity;
        float3 minOverlapAxis = float3.zero;

        for ( int i = 0; i < allAxes.Length; i++ )
        {
            float projMinA = float.MaxValue;
            float projMinB = float.MaxValue;
            float projMaxA = float.MinValue;
            float projMaxB = float.MinValue;

            float3 axis = allAxes[ i ];

            if ( axis.x == 0 && axis.y == 0 && axis.z == 0 )
                continue;

            for ( int j = 0; j < vertsB.Length; j++ )
            {
                float p = math.dot( vertsB[ j ] , axis );

                if ( p < projMinB )
                    projMinB = p;
                if ( p > projMaxB )
                    projMaxB = p;
            }
            for ( int j = 0; j < vertsA.Length; j++ )
            {
                float p = math.dot( vertsA[ j ] , axis );

                if ( p < projMinA )
                    projMinA = p;
                if ( p > projMaxA )
                    projMaxA = p;
            }

            float overlap = GetLineOverLap( projMinA , projMinB , projMaxA , projMaxB );

            if ( overlap <= 0 )
            {
                return;
            }
            else if ( overlap < minOverlap )
            {
                minOverlap = overlap;
                minOverlapAxis = axis;
            }
        }

        float3 dir = originB - originA;
        if ( math.dot( minOverlapAxis , dir ) <= 0 )
            minOverlapAxis *= -1;

        minOverlapAxis = math.normalize( minOverlapAxis );

        float3 displacementA = -( minOverlapAxis * minOverlap ) / 2;
        float3 displacementB = ( minOverlapAxis * minOverlap ) / 2;

        originA += displacementA;
        originB += displacementB;
    }

    // SPHERE CAPSULE
    private void HandleSphereCapsule()
    {
        CollisionSphereCapsule( ref posA , halfSizeA.x , ref posB , rotB , halfSizeB.y , halfSizeB.x );
        DrawSphereCapsule();
    }
    private void CollisionSphereCapsule( ref float3 spherePos , float sphereRadius , ref float3 capsulePos , float3 capsuleRotation , float capsuleLength, float capsuleRadius )
    {
        CapsulePoints capsulePoints = GetCapsuleCollider( capsulePos , capsuleRotation , capsuleLength );
        CapsulePoints capsuleSpheres = GetCapsuleSpheres( capsulePoints.tipPoint , capsulePoints.basePoint , capsuleRadius );

        float3 closestPoint = ClosestPointOnLineSegement( capsuleSpheres.basePoint , capsuleSpheres.tipPoint , spherePos );

        float distance = math.distance( spherePos , closestPoint );
        float penetrationDepth = sphereRadius + capsuleRadius - distance;

        if ( penetrationDepth > 0 )
        {
            ResolveCapsuleCollision( ref spherePos , ref capsulePos , spherePos , closestPoint , sphereRadius , capsuleRadius , distance );
        }
    }

    // SPHERE BOX
    private void HandleSphereBox()
    {
        float3[] verts = GetRotatedVerticesOfBox( posB , halfSizeB , rotB );
        MinMax minMax = GetMinMaxOfOBB( verts );

        CollisionSphereBox( ref posA , halfSizeA.x , ref posB , minMax );
        DrawSphereBox();
    }
    private void CollisionSphereBox( ref float3 spherePos , float sphereRadius , ref float3 boxPos , MinMax minMaxOBB )
    {
        float3 closestPoint = GetClosestPointOnOBBToSphere( spherePos , minMaxOBB );
        float distance = math.distance( closestPoint , spherePos );

        if ( distance < sphereRadius )
        {
            float overlap = sphereRadius - distance;

            float3 collisionDirectionSphere = math.normalize( spherePos - closestPoint );
            float3 collisionDirectionBox = math.normalize( boxPos - closestPoint );

            spherePos += collisionDirectionSphere * ( overlap / 2 );
            boxPos += collisionDirectionBox * ( overlap / 2 );

            Debug.Log( overlap );
        }
    }

    // HELPERS
    private float GetLineOverLap( float min1 , float min2 , float max1 , float max2 )
    {
        float _min1 = math.min( max1 , max2 );
        float _max1 = math.max( min1 , min2 );
        float diff = _min1 - _max1;

        return math.max( 0 , diff );
    }
    private float DegreesToRadians( float degrees )
    {
        return degrees * ( math.PI / 180 );
    }
    private float3[] GetIdentityVerticesOfBox( float3 origin , float3 halfSize )
    {
        float3[] verts = new float3[ 8 ];

        verts[ 0 ] = ( origin + new float3( -halfSize.x , -halfSize.y , -halfSize.z ) );
        verts[ 1 ] = ( origin + new float3( +halfSize.x , -halfSize.y , -halfSize.z ) );
        verts[ 2 ] = ( origin + new float3( +halfSize.x , +halfSize.y , -halfSize.z ) );
        verts[ 3 ] = ( origin + new float3( -halfSize.x , +halfSize.y , -halfSize.z ) );

        verts[ 4 ] = ( origin + new float3( -halfSize.x , -halfSize.y , +halfSize.z ) );
        verts[ 5 ] = ( origin + new float3( +halfSize.x , -halfSize.y , +halfSize.z ) );
        verts[ 6 ] = ( origin + new float3( +halfSize.x , +halfSize.y , +halfSize.z ) );
        verts[ 7 ] = ( origin + new float3( -halfSize.x , +halfSize.y , +halfSize.z ) );

        return verts;
    }
    private float3[] GetRotatedVerticesOfBox( float3 origin , float3 halfSize , float3 rotation )
    {
        float3[] verts = GetIdentityVerticesOfBox( origin , halfSize );

        for ( int i = 0; i < verts.Length; i++ )
            verts[ i ] = RotatePoint3D( verts[ i ] , origin , rotation.y , rotation.z , rotation.x );

        return verts;
    }
    private float3[] GetProjectionAxesOfBox( float3[] vertices )
    {
        float3[] axes = new float3[ 3 ];
        axes[ 0 ] = math.normalize( ( vertices[ 1 ] - vertices[ 0 ] ) );
        axes[ 1 ] = math.normalize( ( vertices[ 3 ] - vertices[ 0 ] ) );
        axes[ 2 ] = math.normalize( ( vertices[ 4 ] - vertices[ 0 ] ) );

        return axes;
    }
    private float3 ClosestPointLineToSphere( float3 spherePos , float sphereRadius , float3 p1 , float3 p2)
    {
        float3 d = spherePos - p1;
        float3 e = p2 - p1;
        float e2 = math.dot( e , e );
        float ed = math.dot( e , d );

        float t = ed / e2;

        if ( t < 0.0f )
            t = 0.0f;
        else if ( t > 1.0f )
            t = 1.0f;

        float3 closestPoint = p1 + e * t;

        return closestPoint;
    }
    private float3 CalculateSphereCollisionDisplacement( float3 pointA , float3 pointB , float radiusA , float radiusB , float distance )
    {
        float overlap = 0.5f * ( distance - radiusA - radiusB );
        float3 displacement = ( overlap * ( pointA - pointB ) ) / ( distance + 0.0001f );

        return displacement;
    }
    private float3 ClosestPointOnLineSegement( float3 a , float3 b , float3 point )
    {
        float3 ab = b - a;
        float t = math.dot( point - a , ab ) / math.dot( ab , ab );
        return a + math.saturate( t ) * ab;
    }
    private float3 RotatePoint3D( float3 point , float3 origin , float pitch , float yaw , float roll )
    {
        float cosa = math.cos( yaw );
        float sina = math.sin( yaw );

        float cosb = math.cos( pitch );
        float sinb = math.sin( pitch );

        float cosc = math.cos( roll );
        float sinc = math.sin( roll );

        float xxA = cosa * cosb;
        float xyA = cosa * sinb * sinc - sina * cosc;
        float xzA = cosa * sinb * cosc + sina * sinc;

        float yxA = sina * cosb;
        float yyA = sina * sinb * sinc + cosa * cosc;
        float yzA = sina * sinb * cosc - cosa * sinc;

        float zxA = -sinb;
        float zyA = cosb * sinc;
        float zzA = cosb * cosc;

        float px = point.x;
        float py = point.y;
        float pz = point.z;

        float pxR = origin.x + xxA * ( px - origin.x ) + xyA * ( py - origin.y ) + xzA * ( pz - origin.z );
        float pyR = origin.y + yxA * ( px - origin.x ) + yyA * ( py - origin.y ) + yzA * ( pz - origin.z );
        float pzR = origin.z + zxA * ( px - origin.x ) + zyA * ( py - origin.y ) + zzA * ( pz - origin.z );

        return new float3( pxR , pyR , pzR );
    }

    private struct MinMax
    {
        public float3 min;
        public float3 max;
    }
    private MinMax GetMinMaxOfOBB( float3[] vertices )
    {
        float xMin = float.MaxValue;
        float yMin = float.MaxValue;
        float zMin = float.MaxValue;

        float xMax = float.MinValue;
        float yMax = float.MinValue;
        float zMax = float.MinValue;

        for ( int i = 0; i < vertices.Length; i++ )
        {
            xMin = math.min( xMin , vertices[ i ].x );
            yMin = math.min( yMin , vertices[ i ].y );
            zMin = math.min( zMin , vertices[ i ].z );

            xMax = math.max( xMax , vertices[ i ].x );
            yMax = math.max( yMax , vertices[ i ].y );
            zMax = math.max( zMax , vertices[ i ].z );
        }

        float3 min = new float3( xMin , yMin , zMin );
        float3 max = new float3( xMax , yMax , zMax );

        return new MinMax
        {
            min = min ,
            max = max
        };
    }
    private float3 GetClosestPointOnOBBToSphere( float3 spherePos , MinMax minMax )
    {
        float x = 0;
        float y = 0;
        float z = 0;

        if ( spherePos.x < minMax.min.x )
            x = minMax.min.x;
        else if ( spherePos.x > minMax.max.x )
            x = minMax.max.x;
        else
            x = spherePos.x;

        if ( spherePos.y < minMax.min.y )
            y = minMax.min.y;
        else if ( spherePos.y > minMax.max.y )
            y = minMax.max.y;
        else
            y = spherePos.y;

        if ( spherePos.z < minMax.min.z )
            z = minMax.min.z;
        else if ( spherePos.z > minMax.max.z )
            z = minMax.max.z;
        else
            z = spherePos.z;

        return new float3( x , y , z );
    }

    // DRAW FUNCTIONS
    private void DrawSpheres()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        shapeA.transform.localScale = halfSizeA * 2;
        shapeB.transform.localScale = halfSizeB * 2;
    }
    private void DrawSphereCapsule()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        float xRotation = DegreesToRadians( 90 );
        shapeB.transform.rotation = quaternion.EulerXYZ( rotB.x + xRotation , rotB.y , rotB.z );

        shapeA.transform.localScale = new float3( halfSizeA.x * 2 , halfSizeA.x * 2 , halfSizeA.x * 2 );
        shapeB.transform.localScale = new float3( halfSizeB.x * 2 , halfSizeB.y * 2 , halfSizeB.x * 2 );
    }
    private void DrawSphereBox()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        shapeB.transform.rotation = quaternion.EulerXYZ( rotB );

        shapeA.transform.localScale = halfSizeA * 2;
        shapeB.transform.localScale = halfSizeB * 2;
    }
    private void DrawCapsuleLock()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        shapeA.transform.localScale = new float3( halfSizeA.x * 2 , halfSizeA.y * 2 , halfSizeA.x * 2 );
        shapeB.transform.localScale = new float3( halfSizeB.x * 2 , halfSizeB.y * 2 , halfSizeB.x * 2 );
    }
    private void DrawCapsules()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        float xRotation = DegreesToRadians( 90 );
        shapeA.transform.rotation = quaternion.EulerXYZ( rotA.x + xRotation , rotA.y , rotA.z );
        shapeB.transform.rotation = quaternion.EulerXYZ( rotB.x + xRotation , rotB.y , rotB.z );

        shapeA.transform.localScale = new float3( halfSizeA.x * 2 , halfSizeA.y * 2 , halfSizeA.x * 2 );
        shapeB.transform.localScale = new float3( halfSizeB.x * 2 , halfSizeB.y * 2 , halfSizeB.x * 2 );
    }
    private void DrawBoxes()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        shapeA.transform.rotation = quaternion.EulerXYZ( rotA );
        shapeB.transform.rotation = quaternion.EulerXYZ( rotB );

        shapeA.transform.localScale = halfSizeA * 2;
        shapeB.transform.localScale = halfSizeB * 2;
    }
}
