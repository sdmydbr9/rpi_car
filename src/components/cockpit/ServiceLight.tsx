import { useState } from 'react';
import { AlertTriangle } from 'lucide-react';
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '../ui/dialog';
import './ServiceLight.css';

export interface SensorStatus {
  front_sonar: 'OK' | 'WARNING' | 'FAILED';
  rear_sonar: 'OK' | 'WARNING' | 'FAILED';
  left_ir: 'OK' | 'WARNING' | 'FAILED';
  right_ir: 'OK' | 'WARNING' | 'FAILED';
}

interface ServiceLightProps {
  isActive: boolean;
  sensorStatus?: SensorStatus;
}

const getStatusIcon = (status: string) => {
  switch (status) {
    case 'OK':
      return '✓';
    case 'WARNING':
      return '!';
    case 'FAILED':
      return '✕';
    default:
      return '✓';
  }
};

const getSensorName = (key: string) => {
  const names: Record<string, string> = {
    front_sonar: 'Front Sonar',
    rear_sonar: 'Rear Sonar',
    left_ir: 'Left IR Sensor',
    right_ir: 'Right IR Sensor',
  };
  return names[key] || key;
};

export const ServiceLight = ({ isActive, sensorStatus }: ServiceLightProps) => {
  const [isOpen, setIsOpen] = useState(false);

  const hasError = sensorStatus
    ? Object.values(sensorStatus).some(status => status !== 'OK')
    : isActive;

  return (
    <Dialog open={isOpen} onOpenChange={setIsOpen}>
      <DialogTrigger asChild>
        <button
          className={`
            w-12 h-12 rounded-full border-2 flex items-center justify-center relative
            transition-all duration-100 touch-feedback font-bold service-light-button
            ${!hasError
              ? 'bg-muted border-muted text-muted-foreground'
              : 'bg-yellow-100 border-yellow-500 text-yellow-400 shadow-lg'
            }
          `}
          title="Service Light - Check sensors"
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 48 48"
            width="22"
            height="22"
            className={hasError ? "text-yellow-400" : "text-muted-foreground"}
          >
            <g>
              <path d="M33.348,26.751c-0.7-0.702-1.932-0.702-2.634,0l-0.44,0.449l-1.818-1.818l6.791-6.798c2.418,0.468,4.855-0.271,6.601-2.015   c2.006-1.997,2.68-5.017,1.717-7.699c-0.063-0.197-0.234-0.341-0.44-0.394c-0.206-0.037-0.431,0.017-0.584,0.179l-4.804,4.804   c-1.52-0.486-2.698-1.672-3.192-3.203l4.811-4.802c0.144-0.162,0.206-0.359,0.162-0.575c-0.044-0.199-0.197-0.379-0.396-0.451   c-2.671-0.952-5.684-0.287-7.69,1.728c-1.744,1.744-2.481,4.19-2.013,6.601l-6.8,6.78l-7.824-7.824l0.215-0.215   c0.217-0.215,0.243-0.556,0.072-0.79l-4.153-5.918c-0.109-0.162-0.28-0.252-0.459-0.271C10.29,4.499,10.11,4.571,9.975,4.698   l-5.27,5.27c-0.136,0.144-0.199,0.324-0.181,0.505c0.009,0.179,0.109,0.341,0.262,0.449l5.918,4.155   c0.107,0.072,0.234,0.107,0.35,0.107c0.162,0,0.324-0.053,0.44-0.179l0.225-0.217l7.824,7.843l-6.789,6.782   c-2.411-0.468-4.857,0.287-6.601,2.015c-1.997,1.997-2.68,5.017-1.728,7.697c0.081,0.199,0.243,0.343,0.449,0.396   c0.217,0.037,0.433-0.018,0.577-0.162l4.811-4.82c1.52,0.503,2.708,1.691,3.194,3.201l-4.804,4.804   c-0.153,0.162-0.215,0.378-0.171,0.575c0.046,0.215,0.199,0.378,0.396,0.449C9.679,43.856,10.524,44,11.351,44   c1.925,0,3.822-0.755,5.216-2.159c1.744-1.728,2.483-4.174,2.015-6.583l6.798-6.8l1.818,1.818l-0.442,0.431   c-0.728,0.737-0.728,1.925,0,2.643l8.769,8.76c0.908,0.917,2.105,1.368,3.291,1.368c1.197,0,2.393-0.451,3.301-1.368   c0.882-0.88,1.367-2.05,1.367-3.291c0-1.241-0.484-2.409-1.367-3.291L33.348,26.751z M10.992,13.764l-4.885-3.437l4.227-4.225   l3.428,4.874L10.992,13.764z M12.593,13.908l1.322-1.313l7.833,7.824l-1.322,1.332L12.593,13.908z M17.306,35.187   c0.503,2.103-0.099,4.262-1.619,5.791c-1.413,1.403-3.444,2.015-5.397,1.691l4.308-4.317c0.153-0.144,0.206-0.359,0.162-0.575   c-0.558-2.249-2.293-3.975-4.524-4.533c-0.217-0.053-0.442,0-0.595,0.162l-4.306,4.299c-0.343-1.943,0.278-3.975,1.698-5.397   c1.512-1.51,3.671-2.122,5.774-1.619c0.206,0.072,0.433,0,0.586-0.162l17.14-17.14c0.153-0.144,0.215-0.378,0.171-0.577   c-0.512-2.103,0.09-4.262,1.61-5.774c1.413-1.42,3.453-2.032,5.397-1.691l-4.299,4.299c-0.162,0.146-0.225,0.378-0.171,0.595   c0.558,2.23,2.293,3.956,4.524,4.514c0.215,0.053,0.44,0,0.593-0.162l4.308-4.299c0.343,1.943-0.278,3.975-1.691,5.397   c-1.52,1.51-3.678,2.122-5.783,1.619c-0.206-0.055-0.431,0-0.584,0.16L17.466,34.61C17.313,34.754,17.251,34.988,17.306,35.187z    M26.254,27.578l1.322-1.313l1.816,1.798l-1.322,1.332L26.254,27.578z M36.399,41.23l-8.76-8.76   c-0.243-0.234-0.243-0.63-0.009-0.882l1.547-1.529l11.574,11.567C39.419,42.543,37.585,42.416,36.399,41.23z M41.633,40.762   L30.048,29.179l1.538-1.547c0.118-0.109,0.28-0.181,0.442-0.181c0.169,0,0.322,0.072,0.44,0.181l8.769,8.778   c0.646,0.628,0.998,1.492,0.998,2.409C42.235,39.521,42.027,40.186,41.633,40.762z"/>
            </g>
            <g>
              <path d="M17.181,32.82c-0.167,0-0.334-0.064-0.461-0.192c-0.253-0.255-0.251-0.667,0.003-0.919l12.798-12.702   c0.255-0.252,0.667-0.251,0.919,0.003c0.253,0.255,0.251,0.667-0.003,0.919L17.639,32.632C17.512,32.757,17.346,32.82,17.181,32.82   z"/>
            </g>
          </svg>
        </button>
      </DialogTrigger>

      {/* Sensor Status Dialog */}
      {hasError && (
        <DialogContent
          className="max-w-full w-[96vw] sm:w-[420px] md:w-[480px] lg:w-[520px] rounded-2xl p-0 border-0 shadow-xl bg-background/95 backdrop-blur-md"
          style={{ minWidth: 0, maxWidth: '100vw', width: '100%', padding: 0 }}
        >
          <div className="flex flex-col w-full h-full max-h-[90vh] min-h-0" style={{ minWidth: 0 }}>
            <div className="flex items-center gap-3 px-4 sm:px-6 pt-5 sm:pt-6 pb-2 border-b border-border/40">
              <span className="inline-flex items-center justify-center w-8 h-8 rounded-full bg-yellow-100 text-yellow-600 shadow-sm">
                <AlertTriangle className="w-6 h-6" />
              </span>
              <div className="flex flex-col">
                <span className="font-semibold text-lg text-yellow-700">Sensor Status Report</span>
                <span className="text-xs text-muted-foreground">System health check</span>
              </div>
            </div>
            <div className="flex-1 overflow-y-auto px-4 sm:px-6 py-4 space-y-3 min-h-[100px] max-h-[50vh]" style={{ minWidth: 0 }}>
              {sensorStatus && Object.entries(sensorStatus).map(([key, status]) => (
                <div
                  key={key}
                  className="flex items-center justify-between p-3 rounded-xl bg-muted/60 border border-border/30 shadow-sm"
                >
                  <span className="font-medium text-sm text-foreground/90">{getSensorName(key)}</span>
                  <div className="flex items-center gap-2">
                    <span
                      className={`text-lg font-bold w-7 h-7 flex items-center justify-center rounded-full border-2 ${
                        status === 'OK'
                          ? 'bg-green-100 border-green-400 text-green-700'
                          : status === 'WARNING'
                          ? 'bg-yellow-100 border-yellow-400 text-yellow-700'
                          : 'bg-red-100 border-red-400 text-red-700'
                      }`}
                    >
                      {getStatusIcon(status)}
                    </span>
                    <span
                      className={`text-xs font-semibold ${
                        status === 'OK'
                          ? 'text-green-700'
                          : status === 'WARNING'
                          ? 'text-yellow-700'
                          : 'text-red-700'
                      }`}
                    >
                      {status}
                    </span>
                  </div>
                </div>
              ))}
            </div>
            <div className="px-4 sm:px-6 pb-4 sm:pb-5 pt-2">
              <div className="flex items-center gap-2 p-3 rounded-xl bg-yellow-50 border border-yellow-200">
                <span className="text-yellow-600 text-lg">⚠️</span>
                <span className="text-sm text-yellow-800">Service light is active. Check sensors before operating the vehicle.</span>
              </div>
            </div>
          </div>
        </DialogContent>
      )}
    </Dialog>
  );
};
