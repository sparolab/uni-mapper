#include <open_lmm/core/dynamic_remover/dynamic_remover_v2.hpp>
#include <open_lmm/server/bootstrap/algorithm_factory.hpp>
#include <open_lmm/utils/config_schema.hpp>
#include <dlfcn.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace open_lmm;
namespace {
using PointCloud = DynamicRemoverBase::PointCloud;
void Check(bool value, const char* message) { if (!value) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); } }
using Count = int (*)(); using Reset = void (*)();
PointCloud::Ptr Scan(float x) { auto scan=std::make_shared<PointCloud>(); pcl::PointXYZI p; p.x=x;p.y=2;p.z=3;p.intensity=4;scan->push_back(p);return scan; }
PointCloud::Ptr DenseScan(float offset) { auto scan=std::make_shared<PointCloud>();for(int x=0;x<20;++x)for(int y=0;y<20;++y){pcl::PointXYZI p;p.x=offset+x*0.3F;p.y=y*0.3F;p.z=(x+y)%7*0.05F;p.intensity=1;scan->push_back(p);}return scan; }
std::vector<std::pair<int,Eigen::Isometry3d>> Poses(){return {{1,Eigen::Isometry3d::Identity()},{0,Eigen::Isometry3d::Identity()}};}
Result<PointCloud::Ptr> Run(const std::string& path,std::string config,const DynamicRemoverBase::RawScanSource& source,const std::shared_ptr<CancellationToken>& token=std::make_shared<CancellationToken>()){
 auto loaded=DynamicRemoverV2::TryLoad(path,config,OPEN_LMM_CAPABILITY_REMOVER_STREAMING_V2);
 if(!loaded)return Result<PointCloud::Ptr>::Failure(loaded.GetError());
 if(!loaded.Value())return Result<PointCloud::Ptr>::Failure(Error::PluginLoadFailed("fixture v2 unavailable"));
 AlgorithmExecutionContext context;context.cancellation=token;
 return (*loaded.Value())->ProcessStreaming(context,{source,Poses(),{}});
}
std::string BuiltinConfig(uint64_t capability) {
 nlohmann::json document={{"dynamic_remover",{{"dynamic_remover_type",capability==OPEN_LMM_CAPABILITY_REMOVER_BATCH_V2?"offline":"online"},{"model",capability==OPEN_LMM_CAPABILITY_REMOVER_BATCH_V2?"free_dom":"otd"}}}};
 auto validated=BuiltinConfigSchemaRegistry().Validate(ConfigDocumentKind::kDynamicRemover,document,"remover ABI fixture");
 Check(validated.IsOk(),"built-in remover fixture config validates");
 return validated.Value().CanonicalJson();
}
}
int main(int argc,char** argv){
 Check(argc==5,"fixture and built-in paths required");const std::string path=argv[1];void* fixture=dlopen(path.c_str(),RTLD_NOW|RTLD_LOCAL);Check(fixture,"fixture dlopen");
 auto count=reinterpret_cast<Count>(dlsym(fixture,"open_lmm_remover_fixture_abort_count"));auto reset=reinterpret_cast<Reset>(dlsym(fixture,"open_lmm_remover_fixture_reset"));Check(count&&reset,"fixture counters");
 DynamicRemoverConfig v2_only;v2_only.type="online";v2_only.model="remover_fixture_v2";v2_only.plugin_abi="v2";v2_only.plugin_config_json="{}";Check(AlgorithmFactory().PreflightRemover(v2_only).IsOk(),"v2-only remover passes preflight without v1 symbols");
 DynamicRemoverBase::RawScanSource shuffled=[](const auto& visitor){auto r=visitor(1,Scan(20));if(!r)return Result<std::size_t>::Failure(r.GetError());r=visitor(0,Scan(10));return r?Result<std::size_t>::Ok(2):Result<std::size_t>::Failure(r.GetError());};
 reset();auto valid=Run(path,"{}",shuffled);if(!valid)std::cerr<<valid.GetError().Message()<<'\n';Check(valid&&valid.Value()->size()==2&&count()==0,"shuffled frames succeed");
 DynamicRemoverBase::RawScanSource missing=[](const auto& visitor){auto r=visitor(0,Scan(1));return r?Result<std::size_t>::Ok(1):Result<std::size_t>::Failure(r.GetError());};
 reset();Check(!Run(path,"{}",missing)&&count()==0,"missing rejected before begin");
 DynamicRemoverBase::RawScanSource duplicate=[](const auto& visitor){auto r=visitor(0,Scan(1));if(!r)return Result<std::size_t>::Failure(r.GetError());r=visitor(0,Scan(2));return r?Result<std::size_t>::Ok(2):Result<std::size_t>::Failure(r.GetError());};
 reset();Check(!Run(path,"{}",duplicate)&&count()==0,"duplicate rejected before begin");
 DynamicRemoverBase::RawScanSource extra=[](const auto& visitor){for(std::size_t i=0;i<3;++i){auto r=visitor(i,Scan(static_cast<float>(i)));if(!r)return Result<std::size_t>::Failure(r.GetError());}return Result<std::size_t>::Ok(3);};
 reset();Check(!Run(path,"{}",extra)&&count()==0,"extra frame rejected before begin");
 DynamicRemoverBase::RawScanSource throws_std=[](const auto&)->Result<std::size_t>{throw std::runtime_error("source failure");};reset();auto std_failure=Run(path,"{}",throws_std);Check(!std_failure&&std_failure.GetError().code==Error::Code::kIoError&&count()==0,"source std exception becomes Result");
 DynamicRemoverBase::RawScanSource throws_unknown=[](const auto&)->Result<std::size_t>{throw 7;};reset();auto unknown_failure=Run(path,"{}",throws_unknown);Check(!unknown_failure&&unknown_failure.GetError().code==Error::Code::kIoError&&count()==0,"source unknown exception becomes Result");
 auto buffering_token=std::make_shared<CancellationToken>();DynamicRemoverBase::RawScanSource cancelled_while_buffering=[&](const auto& visitor){auto r=visitor(0,Scan(1));if(!r)return Result<std::size_t>::Failure(r.GetError());buffering_token->Request();r=visitor(1,Scan(2));return r?Result<std::size_t>::Ok(2):Result<std::size_t>::Failure(r.GetError());};reset();auto buffering_cancelled=Run(path,"{}",cancelled_while_buffering,buffering_token);Check(!buffering_cancelled&&buffering_cancelled.GetError().code==Error::Code::kCancelled&&count()==0,"pre-buffer cancellation precedes begin");
 for(const char* mode:{"fail_begin","fail_push","throw_push","fail_finish","malformed","null_finish"}){reset();Check(!Run(path,mode,shuffled)&&count()==1,"failure aborts once");}
 for(const char* mode:{"slow_begin","slow_push","slow_finish"}){reset();auto token=std::make_shared<CancellationToken>();Result<PointCloud::Ptr> result=Result<PointCloud::Ptr>::Failure(Error::InvalidArgument("not run"));std::thread worker([&]{result=Run(path,mode,shuffled,token);});std::this_thread::sleep_for(std::chrono::milliseconds(10));token->Request();worker.join();Check(!result&&result.GetError().code==Error::Code::kCancelled&&count()==1,"cancellation aborts once");}
 const auto exercise_builtin=[&](const char* library,uint64_t capability){const std::string config=BuiltinConfig(capability);auto loaded=DynamicRemoverV2::TryLoad(library,config,capability);Check(loaded&&loaded.Value(),"built-in v2 load");int admissions=0,releases=0;const auto run_order=[&](bool reverse){DynamicRemoverBase::RawScanSource source=[&](const auto& visitor){for(int n=0;n<2;++n){const std::size_t id=reverse?1-n:n;auto r=visitor(id,DenseScan(static_cast<float>(id)));if(!r)return Result<std::size_t>::Failure(r.GetError());}return Result<std::size_t>::Ok(2);};AlgorithmExecutionContext context;context.cancellation=std::make_shared<CancellationToken>();DynamicRemoverBase::HeavyPhaseAdmission admission=[&](){++admissions;return Result<std::shared_ptr<void>>::Ok(std::shared_ptr<void>(new int(0),[&](void* value){delete static_cast<int*>(value);++releases;}));};return (*loaded.Value())->ProcessStreaming(context,{source,Poses(),admission});};auto ordered=run_order(false);auto reversed=run_order(true);if(!ordered)std::cerr<<library<<" ordered: "<<ordered.GetError().Message()<<'\n';if(!reversed)std::cerr<<library<<" reversed: "<<reversed.GetError().Message()<<'\n';Check(ordered&&reversed&&ordered.Value()&&reversed.Value()&&ordered.Value()->size()==reversed.Value()->size(),"built-in v2 frame-order parity");const int expected=capability==OPEN_LMM_CAPABILITY_REMOVER_BATCH_V2?2:0;Check(admissions==expected&&releases==expected,"batch v2 heavy admission lifetime");for(const auto& p:*ordered.Value())Check(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&std::isfinite(p.intensity),"built-in finite output");};
 exercise_builtin(argv[2],OPEN_LMM_CAPABILITY_REMOVER_STREAMING_V2);
 exercise_builtin(argv[3],OPEN_LMM_CAPABILITY_REMOVER_BATCH_V2);
 auto bad_ops=DynamicRemoverV2::TryLoad(argv[4],"{}",OPEN_LMM_CAPABILITY_REMOVER_STREAMING_V2);Check(!bad_ops,"incomplete remover operation metadata rejected at load");
 dlclose(fixture);std::cout<<"dynamic remover ABI v2 tests passed\n";
}
